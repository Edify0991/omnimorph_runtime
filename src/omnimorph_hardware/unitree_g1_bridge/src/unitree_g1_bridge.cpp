#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include <SharedMemory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "rl_master/hardware/motor_shm_contract.h"
#include "rl_master/kinematics/joint_data.h"

#ifdef OMNIMORPH_HAS_UNITREE_CRC
#include "common/motor_crc_hg.h"
#endif

namespace
{
constexpr size_t kG1MotorCount = 29;
constexpr uint8_t kUnitreeMotorEnable = 1;
constexpr uint8_t kUnitreeMotorDisable = 0;

float fallbackGain(float raw, float fallback)
{
    return raw > 0.0f ? raw : fallback;
}

float decodeSharedMemoryGainByte(uint8_t raw, float fallback)
{
    return raw > 0U ? static_cast<float>(raw) : fallback;
}

bool usesUnitreePdLoop(uint8_t run_mode)
{
    return run_mode == RUN_MODE_R1 || run_mode == RUN_MODE_CSP;
}

bool usesUnitreeTorqueCommand(uint8_t run_mode)
{
    return run_mode == RUN_MODE_CST;
}

float sanitizeFiniteScalar(
    rclcpp::Logger logger,
    const char *field_name,
    size_t motor_index,
    float value)
{
    if (std::isfinite(value))
    {
        return value;
    }
    RCLCPP_WARN(
        logger,
        "non-finite Unitree motor command field '%s' at motor %zu; replacing with 0.0",
        field_name,
        motor_index);
    return 0.0f;
}

std::string summarizeMotorCmd(
    const unitree_hg::msg::LowCmd &cmd,
    size_t joint_count)
{
    std::ostringstream oss;
    const size_t count = std::min(joint_count, cmd.motor_cmd.size());
    for (size_t i = 0; i < count; ++i)
    {
        if (i > 0)
        {
            oss << " | ";
        }
        const auto &mc = cmd.motor_cmd[i];
        oss << "#" << i
            << " mode=" << static_cast<int>(mc.mode)
            << " q=" << mc.q
            << " dq=" << mc.dq
            << " tau=" << mc.tau
            << " kp=" << mc.kp
            << " kd=" << mc.kd;
    }
    return oss.str();
}
} // namespace

class UnitreeG1Bridge final : public rclcpp::Node
{
public:
    UnitreeG1Bridge()
        : Node("unitree_g1_bridge")
    {
        lowstate_topic_ = declare_parameter<std::string>("lowstate_topic", "lowstate");
        lowcmd_topic_ = declare_parameter<std::string>("lowcmd_topic", "/lowcmd");
        control_hz_ = declare_parameter<double>("control_hz", 500.0);
        default_lower_kp_ = declare_parameter<double>("default_lower_kp", 100.0);
        default_lower_kd_ = declare_parameter<double>("default_lower_kd", 1.0);
        default_upper_kp_ = declare_parameter<double>("default_upper_kp", 50.0);
        default_upper_kd_ = declare_parameter<double>("default_upper_kd", 1.0);
        mode_pr_ = declare_parameter<int>("mode_pr", 0);
        disable_when_no_target_ = declare_parameter<bool>("disable_when_no_target", false);
        debug_motor_cmd_cycles_ = declare_parameter<int>("debug_motor_cmd_cycles", 0);
        debug_motor_cmd_joint_count_ = declare_parameter<int>("debug_motor_cmd_joint_count", 6);

        if (control_hz_ <= 0.0)
        {
            throw std::runtime_error("unitree_g1_bridge control_hz must be positive");
        }

        connectSharedMemory();

        lowstate_sub_ = create_subscription<unitree_hg::msg::LowState>(
            lowstate_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                handleLowState(msg);
            });

        lowcmd_pub_ = create_publisher<unitree_hg::msg::LowCmd>(
            lowcmd_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / control_hz_));
        timer_ = create_wall_timer(period, [this] { publishLowCmd(); });

        RCLCPP_INFO(
            get_logger(),
            "Unitree G1 bridge active: shm target='%s', shm feedback='%s', lowstate='%s', lowcmd='%s'",
            rl_master::hardware::kMotorTargetShmPath,
            rl_master::hardware::kMotorFeedbackShmPath,
            lowstate_topic_.c_str(),
            lowcmd_topic_.c_str());
    }

private:
    using MotorHandle = rl_master::hardware::MotorHandle;

    void connectSharedMemory()
    {
        shm_target_ = std::make_unique<SharedMemory>(
            rl_master::hardware::kMotorTargetShmPath,
            sizeof(MotorHandle) * rl_master::hardware::kMotorShmSlotCount,
            rl_master::hardware::kMotorTargetShmKeyNum,
            LOCK_TYPE_MUTEX,
            rl_master::hardware::kMotorTargetShmSemName);
        shm_feedback_ = std::make_unique<SharedMemory>(
            rl_master::hardware::kMotorFeedbackShmPath,
            sizeof(MotorHandle) * rl_master::hardware::kMotorShmSlotCount,
            rl_master::hardware::kMotorFeedbackShmKeyNum,
            LOCK_TYPE_MUTEX,
            rl_master::hardware::kMotorFeedbackShmSemName);

        shm_target_->connect();
        shm_feedback_->connect();
    }

    void handleLowState(const unitree_hg::msg::LowState::SharedPtr &msg)
    {
        mode_machine_ = static_cast<int>(msg->mode_machine);

        feedback_slots_.fill(MotorHandle{});
        const size_t available = std::min(kG1MotorCount, msg->motor_state.size());
        for (size_t i = 0; i < available; ++i)
        {
            feedback_slots_[i].io.feedback.feedback_pos = msg->motor_state[i].q;
            feedback_slots_[i].io.feedback.feedback_speed = msg->motor_state[i].dq;
            feedback_slots_[i].io.feedback.feedback_torque = msg->motor_state[i].tau_est;
        }

        try
        {
            shm_feedback_->write(
                feedback_slots_.data(),
                static_cast<int>(rl_master::hardware::kMotorShmSlotCount),
                0);
            has_lowstate_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "failed to write Unitree feedback shared memory: %s",
                e.what());
        }
    }

    void publishLowCmd()
    {
        try
        {
            shm_target_->read(
                target_slots_.data(),
                static_cast<int>(rl_master::hardware::kMotorShmSlotCount),
                0);
            has_target_ = true;
        }
        catch (const std::exception &e)
        {
            has_target_ = false;
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "failed to read Unitree target shared memory: %s",
                e.what());
        }

        unitree_hg::msg::LowCmd cmd;
        cmd.mode_pr = static_cast<uint8_t>(mode_pr_);
        cmd.mode_machine = static_cast<uint8_t>(mode_machine_);

        const size_t available = std::min(kG1MotorCount, cmd.motor_cmd.size());
        for (size_t i = 0; i < available; ++i)
        {
            const auto &target = target_slots_[i];
            const bool enabled = has_target_ || !disable_when_no_target_;
            const bool active_mode = target.run_mode != 0;
            const bool lower_body = i < 15;
            const bool pd_loop = usesUnitreePdLoop(target.run_mode);
            const bool torque_loop = usesUnitreeTorqueCommand(target.run_mode);

            cmd.motor_cmd[i].mode = (enabled && active_mode) ? kUnitreeMotorEnable : kUnitreeMotorDisable;
            cmd.motor_cmd[i].q = sanitizeFiniteScalar(
                get_logger(),
                "q",
                i,
                pd_loop ? target.io.target.target_pos : 0.0f);
            cmd.motor_cmd[i].dq = sanitizeFiniteScalar(
                get_logger(),
                "dq",
                i,
                pd_loop ? target.io.target.target_speed : 0.0f);
            cmd.motor_cmd[i].tau = sanitizeFiniteScalar(
                get_logger(),
                "tau",
                i,
                torque_loop ? target.io.target.target_torque : 0.0f);
            cmd.motor_cmd[i].kp = sanitizeFiniteScalar(
                get_logger(),
                "kp",
                i,
                pd_loop
                    ? decodeSharedMemoryGainByte(
                          target.reserved[0],
                          static_cast<float>(lower_body ? default_lower_kp_ : default_upper_kp_))
                    : 0.0f);
            cmd.motor_cmd[i].kd = sanitizeFiniteScalar(
                get_logger(),
                "kd",
                i,
                pd_loop
                    ? decodeSharedMemoryGainByte(
                          target.reserved[1],
                          static_cast<float>(lower_body ? default_lower_kd_ : default_upper_kd_))
                    : 0.0f);
        }

#ifdef OMNIMORPH_HAS_UNITREE_CRC
        get_crc(cmd);
#else
        RCLCPP_WARN_ONCE(
            get_logger(),
            "Unitree CRC helper was not found at build time; source official unitree_ros2 and rebuild before real hardware use.");
#endif

        if (debug_publish_count_ < debug_motor_cmd_cycles_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Unitree LowCmd sample %d/%d: %s",
                debug_publish_count_ + 1,
                debug_motor_cmd_cycles_,
                summarizeMotorCmd(
                    cmd,
                    static_cast<size_t>(std::max(0, debug_motor_cmd_joint_count_)))
                    .c_str());
            ++debug_publish_count_;
        }

        lowcmd_pub_->publish(cmd);
        if (!has_lowstate_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "no Unitree LowState received yet; publishing commands with feedback shared memory still empty");
        }
    }

    std::string lowstate_topic_;
    std::string lowcmd_topic_;
    double control_hz_ = 500.0;
    double default_lower_kp_ = 100.0;
    double default_lower_kd_ = 1.0;
    double default_upper_kp_ = 50.0;
    double default_upper_kd_ = 1.0;
    int mode_pr_ = 0;
    int mode_machine_ = 0;
    bool disable_when_no_target_ = false;
    bool has_target_ = false;
    bool has_lowstate_ = false;
    int debug_motor_cmd_cycles_ = 0;
    int debug_motor_cmd_joint_count_ = 6;
    int debug_publish_count_ = 0;

    std::array<MotorHandle, rl_master::hardware::kMotorShmSlotCount> target_slots_{};
    std::array<MotorHandle, rl_master::hardware::kMotorShmSlotCount> feedback_slots_{};
    std::unique_ptr<SharedMemory> shm_target_;
    std::unique_ptr<SharedMemory> shm_feedback_;

    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeG1Bridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
