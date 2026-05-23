#include "rl_master/solver/motor_io_backend.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <mutex>
#include <system_error>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <unitree_hg/msg/low_cmd.hpp>
#include <unitree_hg/msg/low_state.hpp>

#include "rl_master/kinematics/joint_data.h"
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

float fallbackGain(float raw, float fallback)
{
    return raw > 0.0f ? raw : fallback;
}

class UnitreeG1DdsMotorIoBackend final : public MotorIoBackend
{
public:
    std::string backendId() const override
    {
        return "unitree_g1_dds";
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
        node_->get_parameter("unitree_lowstate_topic", lowstate_topic_);
        node_->get_parameter("unitree_lowcmd_topic", lowcmd_topic_);
        node_->get_parameter("unitree_mode_pr", mode_pr_);
        node_->get_parameter("unitree_lowstate_timeout_sec", lowstate_timeout_sec_);
        node_->get_parameter("unitree_default_lower_kp", default_lower_kp_);
        node_->get_parameter("unitree_default_lower_kd", default_lower_kd_);
        node_->get_parameter("unitree_default_upper_kp", default_upper_kp_);
        node_->get_parameter("unitree_default_upper_kd", default_upper_kd_);

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

            cmd.motor_cmd[i].mode = active_mode ? kUnitreeMotorEnable : kUnitreeMotorDisable;
            cmd.motor_cmd[i].q = pd_loop ? slot.io.target.target_pos : 0.0f;
            cmd.motor_cmd[i].dq = pd_loop ? slot.io.target.target_speed : 0.0f;
            cmd.motor_cmd[i].tau = slot.io.target.target_torque;
            cmd.motor_cmd[i].kp = pd_loop
                                      ? fallbackGain(
                                            static_cast<float>(slot.pd[0]),
                                            static_cast<float>(lower_body ? default_lower_kp_ : default_upper_kp_))
                                      : 0.0f;
            cmd.motor_cmd[i].kd = pd_loop
                                      ? fallbackGain(
                                            static_cast<float>(slot.pd[1]),
                                            static_cast<float>(lower_body ? default_lower_kd_ : default_upper_kd_))
                                      : 0.0f;
        }

#ifdef OMNIMORPH_HAS_UNITREE_CRC
        get_crc(cmd);
#else
        RCLCPP_WARN_ONCE(
            node_->get_logger(),
            "Unitree CRC helper was not found at build time; set UNITREE_ROS2_ROOT and rebuild before real hardware use.");
#endif

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

    std::mutex feedback_mutex_;
    std::array<MotorHandle, kMotorShmSlotCount> latest_feedback_{};
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
