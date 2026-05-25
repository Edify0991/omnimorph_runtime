#include "rl_master/solver/motor_io_backend.h"

#ifdef RL_MASTER_HAS_UNITREE_SDK2

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <system_error>
#include <thread>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "rl_master/kinematics/joint_data.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/rl_protocol.h"
#include "rl_master/runtime/realtime_utils.h"
#include "rl_master/solver/unitree_sdk2_support.h"

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

float sanitizeFiniteScalar(
    const char *field_name,
    size_t motor_index,
    float value)
{
    if (std::isfinite(value))
    {
        return value;
    }
    std::cerr << "[RL_solver][unitree_sdk2] non-finite command field '" << field_name
              << "' at motor " << motor_index << ", replacing with 0.0" << std::endl;
    return 0.0f;
}

class UnitreeG1Sdk2MotorIoBackend final : public MotorIoBackend
{
public:
    std::string backendId() const override
    {
        return "unitree_g1_sdk2";
    }

    void updateSourceContract(const SourceContract &source_contract) override
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        sdk2_cfg_ = source_contract.unitree_sdk2;
    }

    void connect() override
    {
        SourceContractUnitreeSdk2 cfg;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            cfg = sdk2_cfg_;
        }

        ensureUnitreeSdk2ChannelFactoryInitialized(cfg);

        lowcmd_pub_ = std::make_shared<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(
            cfg.lowcmd_topic);
        lowcmd_pub_->InitChannel();

        lowstate_sub_ = std::make_shared<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(
            cfg.lowstate_topic);
        lowstate_sub_->InitChannel(
            [this](const void *message) { this->handleLowState(message); },
            cfg.queue_len);

        stop_requested_.store(false);
        try
        {
            writer_thread_ = std::thread([this]() { writerLoop(); });
        }
        catch (const std::system_error &e)
        {
            throw std::runtime_error(
                std::string("failed to start Unitree SDK2 writer thread: ") + e.what());
        }

        std::cout << "[RL_solver] Unitree SDK2 motor IO active: "
                  << summarizeUnitreeSdk2Config(cfg) << std::endl;
    }

    ~UnitreeG1Sdk2MotorIoBackend() override
    {
        stop_requested_.store(true);
        if (writer_thread_.joinable())
        {
            writer_thread_.join();
        }
        lowstate_sub_.reset();
        lowcmd_pub_.reset();
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
        const float timeout_sec = currentLowstateTimeoutSec();
        if (!has_lowstate_.load())
        {
            maybeWarn(now, "waiting for Unitree SDK2 LowState");
            return;
        }
        if ((now - latest_lowstate_time_sec_.load()) > timeout_sec)
        {
            maybeWarn(now, "Unitree SDK2 LowState stale");
        }
    }

    void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target) override
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        latest_target_ = target;
    }

private:
    void handleLowState(const void *message)
    {
        if (!message)
        {
            return;
        }

        const auto &msg = *static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
        std::array<MotorHandle, kMotorShmSlotCount> feedback{};
        mode_machine_.store(static_cast<int>(msg.mode_machine()));

        const size_t available = std::min(kG1MotorCount, msg.motor_state().size());
        for (size_t i = 0; i < available; ++i)
        {
            const auto &motor_state = msg.motor_state()[i];
            feedback[i].io.feedback.feedback_pos = motor_state.q();
            feedback[i].io.feedback.feedback_speed = motor_state.dq();
            feedback[i].io.feedback.feedback_torque = motor_state.tau_est();
        }

        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            latest_feedback_ = feedback;
        }
        latest_lowstate_time_sec_.store(rl_master::monotonicTimeSec());
        has_lowstate_.store(true);
    }

    void writerLoop()
    {
        while (!stop_requested_.load())
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            if (lowcmd_pub_)
            {
                unitree_hg::msg::dds_::LowCmd_ cmd = buildLowCmd();
                if (!lowcmd_pub_->Write(cmd))
                {
                    maybeWarn(rl_master::monotonicTimeSec(), "Unitree SDK2 lowcmd write failed");
                }
            }

            const int writer_period_us = currentWriterPeriodUs();
            std::this_thread::sleep_until(cycle_start + std::chrono::microseconds(writer_period_us));
        }
    }

    unitree_hg::msg::dds_::LowCmd_ buildLowCmd()
    {
        SourceContractUnitreeSdk2 cfg;
        std::array<MotorHandle, kMotorShmSlotCount> target{};
        {
            std::lock_guard<std::mutex> config_lock(config_mutex_);
            cfg = sdk2_cfg_;
        }
        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            target = latest_target_;
        }

        unitree_hg::msg::dds_::LowCmd_ cmd;
        cmd.mode_pr(static_cast<uint8_t>(cfg.mode_pr));
        cmd.mode_machine(static_cast<uint8_t>(mode_machine_.load()));

        const size_t available = std::min(kG1MotorCount, cmd.motor_cmd().size());
        for (size_t i = 0; i < available; ++i)
        {
            auto &motor_cmd = cmd.motor_cmd()[i];
            const auto &slot = target[i];
            const bool active_mode = slot.run_mode != 0;
            const bool lower_body = i < 15;
            const bool pd_loop = usesUnitreePdLoop(slot.run_mode);

            motor_cmd.mode(active_mode ? kUnitreeMotorEnable : kUnitreeMotorDisable);
            motor_cmd.q(sanitizeFiniteScalar(
                "q",
                i,
                pd_loop ? slot.io.target.target_pos : 0.0f));
            motor_cmd.dq(sanitizeFiniteScalar(
                "dq",
                i,
                pd_loop ? slot.io.target.target_speed : 0.0f));
            motor_cmd.tau(sanitizeFiniteScalar(
                "tau",
                i,
                slot.io.target.target_torque));
            motor_cmd.kp(sanitizeFiniteScalar(
                "kp",
                i,
                pd_loop
                    ? fallbackGain(slot.pd[0], lower_body ? cfg.default_lower_kp : cfg.default_upper_kp)
                    : 0.0f));
            motor_cmd.kd(sanitizeFiniteScalar(
                "kd",
                i,
                pd_loop
                    ? fallbackGain(slot.pd[1], lower_body ? cfg.default_lower_kd : cfg.default_upper_kd)
                    : 0.0f));
            motor_cmd.reserve(0U);
        }

        cmd.crc(crc32_core(
            reinterpret_cast<uint32_t *>(&cmd),
            static_cast<uint32_t>((sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1)));
        return cmd;
    }

    int currentWriterPeriodUs() const
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return sdk2_cfg_.writer_period_us;
    }

    float currentLowstateTimeoutSec() const
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        return sdk2_cfg_.lowstate_timeout_sec;
    }

    void maybeWarn(double now_sec, const std::string &message)
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        if ((now_sec - last_warn_time_sec_) < 2.0)
        {
            return;
        }
        last_warn_time_sec_ = now_sec;
        std::cerr << "[RL_solver][unitree_sdk2] " << message << std::endl;
    }

    mutable std::mutex config_mutex_;
    SourceContractUnitreeSdk2 sdk2_cfg_{};

    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> lowcmd_pub_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> lowstate_sub_;

    std::mutex feedback_mutex_;
    std::array<MotorHandle, kMotorShmSlotCount> latest_feedback_{};

    std::mutex target_mutex_;
    std::array<MotorHandle, kMotorShmSlotCount> latest_target_{};

    std::atomic<bool> stop_requested_{false};
    std::thread writer_thread_;
    std::atomic<bool> has_lowstate_{false};
    std::atomic<double> latest_lowstate_time_sec_{0.0};
    std::atomic<int> mode_machine_{0};

    mutable std::mutex log_mutex_;
    double last_warn_time_sec_ = 0.0;
};

} // namespace

std::unique_ptr<MotorIoBackend> createUnitreeG1Sdk2MotorIoBackend()
{
    return std::make_unique<UnitreeG1Sdk2MotorIoBackend>();
}

} // namespace rl_master::solver

#endif
