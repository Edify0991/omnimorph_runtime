#include "rl_master/solver/motor_io_backend.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <system_error>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "rl_master/kinematics/joint_data.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/rl_protocol.h"
#include "rl_master/runtime/realtime_utils.h"

#ifdef OMNIMORPH_HAS_UNITREE_CRC
#include "common/motor_crc_hg.h"
#endif

namespace rl_master::solver
{
namespace
{
constexpr size_t kG1MotorCount = 29;
constexpr uint8_t kUnitreeMotorEnable = 1;
constexpr uint8_t kUnitreeMotorDisable = 0;

bool usesUnitreePdLoop(uint8_t run_mode)
{
    return run_mode == RUN_MODE_R1 || run_mode == RUN_MODE_CSP;
}

bool usesUnitreeTorqueCommand(uint8_t run_mode)
{
    return run_mode == RUN_MODE_CST;
}

float fallbackGain(float raw, float fallback)
{
    return raw > 0.0f ? raw : fallback;
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

class UnitreeG1DdsMotorIoBackend final : public MotorIoBackend
{
public:
    std::string backendId() const override
    {
        return "unitree_g1_dds";
    }

    void updateSourceContract(const SourceContract &source_contract) override
    {
        const SourceContractUnitreeRos2 cfg = source_contract.unitree_ros2;
        lowstate_topic_ = cfg.lowstate_topic;
        lowcmd_topic_ = cfg.lowcmd_topic;
        mode_pr_ = cfg.mode_pr;
        lowstate_timeout_sec_ = static_cast<double>(cfg.lowstate_timeout_sec);
        default_lower_kp_ = static_cast<double>(cfg.default_lower_kp);
        default_lower_kd_ = static_cast<double>(cfg.default_lower_kd);
        default_upper_kp_ = static_cast<double>(cfg.default_upper_kp);
        default_upper_kd_ = static_cast<double>(cfg.default_upper_kd);
        debug_motor_cmd_cycles_ = cfg.debug_motor_cmd_cycles;
        debug_motor_cmd_joint_count_ = cfg.debug_motor_cmd_joint_count;
    }

    void writePdGains(size_t motor_index, MotorHandle *target, const JointData &joint_cmd) override
    {
        if (target)
        {
            target->reserved[0] = 0U;
            target->reserved[1] = 0U;
        }
        if (motor_index >= commanded_kp_.size() || motor_index >= commanded_kd_.size())
        {
            return;
        }
        if (joint_cmd.mode == RUN_MODE_R1 || joint_cmd.mode == RUN_MODE_CSP)
        {
            commanded_kp_[motor_index] = joint_cmd.kp;
            commanded_kd_[motor_index] = joint_cmd.kd;
            return;
        }
        commanded_kp_[motor_index] = 0.0f;
        commanded_kd_[motor_index] = 0.0f;
    }

    void connect() override
    {
        if (!rclcpp::ok())
        {
            int argc = 0;
            char **argv = nullptr;
            rclcpp::init(argc, argv);
        }

        node_ = std::make_shared<rclcpp::Node>("rl_solver_unitree_g1_motor_io");
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        node_->declare_parameter<std::string>("unitree_lowstate_topic", lowstate_topic_);
        node_->declare_parameter<std::string>("unitree_lowcmd_topic", lowcmd_topic_);
        node_->declare_parameter<int>("unitree_mode_pr", mode_pr_);
        node_->declare_parameter<double>("unitree_lowstate_timeout_sec", lowstate_timeout_sec_);
        node_->declare_parameter<double>("unitree_default_lower_kp", default_lower_kp_);
        node_->declare_parameter<double>("unitree_default_lower_kd", default_lower_kd_);
        node_->declare_parameter<double>("unitree_default_upper_kp", default_upper_kp_);
        node_->declare_parameter<double>("unitree_default_upper_kd", default_upper_kd_);
        node_->declare_parameter<int>("unitree_debug_motor_cmd_cycles", debug_motor_cmd_cycles_);
        node_->declare_parameter<int>("unitree_debug_motor_cmd_joint_count", debug_motor_cmd_joint_count_);
        node_->get_parameter("unitree_lowstate_topic", lowstate_topic_);
        node_->get_parameter("unitree_lowcmd_topic", lowcmd_topic_);
        node_->get_parameter("unitree_mode_pr", mode_pr_);
        node_->get_parameter("unitree_lowstate_timeout_sec", lowstate_timeout_sec_);
        node_->get_parameter("unitree_default_lower_kp", default_lower_kp_);
        node_->get_parameter("unitree_default_lower_kd", default_lower_kd_);
        node_->get_parameter("unitree_default_upper_kp", default_upper_kp_);
        node_->get_parameter("unitree_default_upper_kd", default_upper_kd_);
        node_->get_parameter("unitree_debug_motor_cmd_cycles", debug_motor_cmd_cycles_);
        node_->get_parameter("unitree_debug_motor_cmd_joint_count", debug_motor_cmd_joint_count_);

        lowstate_sub_ = node_->create_subscription<unitree_hg::msg::LowState>(
            lowstate_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
            [this](const unitree_hg::msg::LowState::SharedPtr msg) {
                this->handleLowState(msg);
            });

        lowcmd_pub_ = node_->create_publisher<unitree_hg::msg::LowCmd>(
            lowcmd_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        executor_->add_node(node_);
        try
        {
            spin_thread_ = std::thread([this] {
                executor_->spin();
            });
        }
        catch (const std::system_error &e)
        {
            throw std::runtime_error(
                std::string("failed to start Unitree G1 motor IO executor thread: ") + e.what());
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Unitree G1 in-process motor IO active: lowstate='%s', lowcmd='%s', mode_pr=%d",
            lowstate_topic_.c_str(),
            lowcmd_topic_.c_str(),
            mode_pr_);
    }

    ~UnitreeG1DdsMotorIoBackend() override
    {
        if (executor_)
        {
            executor_->cancel();
        }
        if (spin_thread_.joinable())
        {
            spin_thread_.join();
        }
        if (executor_ && node_)
        {
            executor_->remove_node(node_);
        }
    }

    void readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback) override
    {
        if (!feedback)
        {
            return;
        }
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            *feedback = latest_feedback_;
        }

        const double now = rl_master::monotonicTimeSec();
        if (!has_lowstate_)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "waiting for Unitree LowState on '%s'",
                lowstate_topic_.c_str());
            return;
        }
        if ((now - latest_lowstate_time_sec_) > lowstate_timeout_sec_)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                2000,
                "Unitree LowState stale: age=%.3f sec",
                now - latest_lowstate_time_sec_);
        }
    }

    void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target) override
    {
        if (!lowcmd_pub_)
        {
            throw std::runtime_error("UnitreeG1DdsMotorIoBackend::writeTarget called before connect().");
        }

        unitree_hg::msg::LowCmd cmd;
        cmd.mode_pr = static_cast<uint8_t>(mode_pr_);
        cmd.mode_machine = static_cast<uint8_t>(mode_machine_);

        const size_t available = std::min(kG1MotorCount, cmd.motor_cmd.size());
        for (size_t i = 0; i < available; ++i)
        {
            const auto &slot = target[i];
            const bool active_mode = slot.run_mode != 0;
            const bool lower_body = i < 15;
            const bool pd_loop = usesUnitreePdLoop(slot.run_mode);
            const bool torque_loop = usesUnitreeTorqueCommand(slot.run_mode);

            cmd.motor_cmd[i].mode = active_mode ? kUnitreeMotorEnable : kUnitreeMotorDisable;
            cmd.motor_cmd[i].q = sanitizeFiniteScalar(
                node_->get_logger(),
                "q",
                i,
                pd_loop ? slot.io.target.target_pos : 0.0f);
            cmd.motor_cmd[i].dq = sanitizeFiniteScalar(
                node_->get_logger(),
                "dq",
                i,
                pd_loop ? slot.io.target.target_speed : 0.0f);
            cmd.motor_cmd[i].tau = sanitizeFiniteScalar(
                node_->get_logger(),
                "tau",
                i,
                torque_loop ? slot.io.target.target_torque : 0.0f);
            cmd.motor_cmd[i].kp = sanitizeFiniteScalar(
                node_->get_logger(),
                "kp",
                i,
                pd_loop
                    ? fallbackGain(
                          commanded_kp_[i],
                          static_cast<float>(lower_body ? default_lower_kp_ : default_upper_kp_))
                    : 0.0f);
            cmd.motor_cmd[i].kd = sanitizeFiniteScalar(
                node_->get_logger(),
                "kd",
                i,
                pd_loop
                    ? fallbackGain(
                          commanded_kd_[i],
                          static_cast<float>(lower_body ? default_lower_kd_ : default_upper_kd_))
                    : 0.0f);
        }

#ifdef OMNIMORPH_HAS_UNITREE_CRC
        get_crc(cmd);
#else
        RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Unitree CRC helper was not found at build time; set UNITREE_ROS2_ROOT and rebuild before real hardware use.");
#endif

        if (debug_publish_count_ < debug_motor_cmd_cycles_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
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
    }

private:
    void handleLowState(const unitree_hg::msg::LowState::SharedPtr &msg)
    {
        std::array<MotorHandle, kMotorShmSlotCount> feedback{};
        mode_machine_ = static_cast<int>(msg->mode_machine);

        const size_t available = std::min(kG1MotorCount, msg->motor_state.size());
        for (size_t i = 0; i < available; ++i)
        {
            feedback[i].io.feedback.feedback_pos = msg->motor_state[i].q;
            feedback[i].io.feedback.feedback_speed = msg->motor_state[i].dq;
            feedback[i].io.feedback.feedback_torque = msg->motor_state[i].tau_est;
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_ = feedback;
            latest_lowstate_time_sec_ = rl_master::monotonicTimeSec();
            has_lowstate_ = true;
        }
    }

    std::string lowstate_topic_ = "lowstate";
    std::string lowcmd_topic_ = "/lowcmd";
    int mode_pr_ = 0;
    int mode_machine_ = 0;
    double lowstate_timeout_sec_ = 0.1;
    double default_lower_kp_ = 100.0;
    double default_lower_kd_ = 1.0;
    double default_upper_kp_ = 50.0;
    double default_upper_kd_ = 1.0;
    int debug_motor_cmd_cycles_ = 0;
    int debug_motor_cmd_joint_count_ = 6;
    int debug_publish_count_ = 0;

    std::mutex feedback_mutex_;
    std::array<MotorHandle, kMotorShmSlotCount> latest_feedback_{};
    std::array<float, kMotorShmSlotCount> commanded_kp_{};
    std::array<float, kMotorShmSlotCount> commanded_kd_{};
    double latest_lowstate_time_sec_ = 0.0;
    bool has_lowstate_ = false;

    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr lowstate_sub_;
    rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr lowcmd_pub_;
};

} // namespace

std::unique_ptr<MotorIoBackend> createUnitreeG1DdsMotorIoBackend()
{
    return std::make_unique<UnitreeG1DdsMotorIoBackend>();
}

} // namespace rl_master::solver
