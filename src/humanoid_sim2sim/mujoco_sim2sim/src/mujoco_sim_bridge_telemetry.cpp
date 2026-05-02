#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{

void MujocoSimBridge::startStateTelemetry()
{
    stopStateTelemetry();
    io_stop_requested_.store(false);
    state_telemetry_thread_ = std::thread([this]() {
        while (!io_stop_requested_.load())
        {
            rl_master::RobotStateData state;
            bool enabled = false;
            double publish_hz = 0.0;
            bool has_state = false;

            {
                std::unique_lock<std::mutex> lock(telemetry_mutex_);
                enabled = enable_state_telemetry_;
                publish_hz = state_telemetry_hz_;
                has_state = has_mirrored_state_;
                if (!enabled || publish_hz <= 0.0 || !has_state)
                {
                    telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_state_telemetry_ && state_telemetry_hz_ > 0.0 && has_mirrored_state_);
                        });
                    continue;
                }
                state = latest_mirrored_state_;
            }

            publishRobotState(state);

            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            telemetry_cv_.wait_for(
                lock,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / publish_hz)),
                [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopStateTelemetry()
{
    io_stop_requested_.store(true);
    telemetry_cv_.notify_all();
    if (state_telemetry_thread_.joinable())
    {
        state_telemetry_thread_.join();
    }
}

void MujocoSimBridge::startViewerTelemetry()
{
    stopViewerTelemetry();
    io_stop_requested_.store(false);
    viewer_telemetry_thread_ = std::thread([this]() {
        auto last_frame_pub = std::chrono::steady_clock::time_point{};
        auto last_inspector_pub = std::chrono::steady_clock::time_point{};

        while (!io_stop_requested_.load())
        {
            std::vector<float> qpos;
            std::vector<float> qvel;
            std::vector<float> ctrl;
            float sim_time = 0.0f;
            std::string inspector_text;
            bool has_frame = false;
            bool has_inspector = false;

            {
                std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
                if ((!enable_python_viewer_stream_ || !has_viewer_frame_) &&
                    (!enable_python_viewer_inspector_ || !has_viewer_inspector_))
                {
                    viewer_telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_python_viewer_stream_ && has_viewer_frame_) ||
                                   (enable_python_viewer_inspector_ && has_viewer_inspector_);
                        });
                    continue;
                }

                has_frame = has_viewer_frame_;
                has_inspector = has_viewer_inspector_;
                if (has_frame)
                {
                    qpos = latest_viewer_qpos_;
                    qvel = latest_viewer_qvel_;
                    ctrl = latest_viewer_ctrl_;
                    sim_time = latest_viewer_sim_time_;
                }
                if (has_inspector)
                {
                    inspector_text = latest_viewer_inspector_text_;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (enable_python_viewer_stream_ && has_frame)
            {
                const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, viewer_fps_));
                if (last_frame_pub.time_since_epoch().count() == 0 ||
                    (now - last_frame_pub) >= frame_period)
                {
                    publishViewerFrameMirror(qpos, qvel, ctrl, sim_time);
                    last_frame_pub = now;
                }
            }

            if (enable_python_viewer_inspector_ && has_inspector)
            {
                const auto inspector_period = std::chrono::duration<double>(1.0 / std::max(0.1, viewer_inspector_hz_));
                if (last_inspector_pub.time_since_epoch().count() == 0 ||
                    (now - last_inspector_pub) >= inspector_period)
                {
                    publishViewerInspectorText(inspector_text);
                    last_inspector_pub = now;
                }
            }

            std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
            viewer_telemetry_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopViewerTelemetry()
{
    io_stop_requested_.store(true);
    viewer_telemetry_cv_.notify_all();
    if (viewer_telemetry_thread_.joinable())
    {
        viewer_telemetry_thread_.join();
    }
}

void MujocoSimBridge::updateMirroredState(const rl_master::RobotStateData &state)
{
    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        should_notify = !has_mirrored_state_;
        latest_mirrored_state_ = state;
        has_mirrored_state_ = true;
    }
    if (should_notify)
    {
        telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::updateViewerFrameMirror()
{
    if (!enable_python_viewer_stream_)
    {
        return;
    }

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_frame_;
        latest_viewer_qpos_.assign(model_->nq, 0.0f);
        latest_viewer_qvel_.assign(model_->nv, 0.0f);
        latest_viewer_ctrl_.assign(model_->nu, 0.0f);
        for (int i = 0; i < model_->nq; ++i)
        {
            latest_viewer_qpos_[i] = static_cast<float>(data_->qpos[i]);
        }
        for (int i = 0; i < model_->nv; ++i)
        {
            latest_viewer_qvel_[i] = static_cast<float>(data_->qvel[i]);
        }
        for (int i = 0; i < model_->nu; ++i)
        {
            latest_viewer_ctrl_[i] = static_cast<float>(data_->ctrl[i]);
        }
        latest_viewer_sim_time_ = static_cast<float>(data_->time);
        has_viewer_frame_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::updateViewerInspectorMirror(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    if (!enable_python_viewer_inspector_)
    {
        return;
    }

    double mean_abs_joint_error = 0.0;
    double max_abs_joint_error = 0.0;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const double err = std::abs(static_cast<double>(last_target_q_[i]) - static_cast<double>(state.joint_q[i]));
        mean_abs_joint_error += err;
        max_abs_joint_error = std::max(max_abs_joint_error, err);
    }
    if (!joint_names_.empty())
    {
        mean_abs_joint_error /= static_cast<double>(joint_names_.size());
    }

    std::ostringstream oss;
    const rl_master::TeleopCommand teleop_command = latestTeleopCommand();
    oss << std::fixed << std::setprecision(3)
        << "mode_id=" << controller_runtime_.activeModeId()
        << " section=" << controller_runtime_.activeConfigSection()
        << " runtime=" << rl_master::commandRuntimeModeName(runtime_mode.mode)
        << " active=" << (runtime_mode.open_rl_active ? "1" : "0")
        << " open_rl=" << static_cast<double>(command.open_rl)
        << " sim_t=" << static_cast<double>(data_->time)
        << " teleop=("
        << static_cast<double>(teleop_command.vx) << ","
        << static_cast<double>(teleop_command.vy) << ","
        << static_cast<double>(teleop_command.dyaw) << ")"
        << " base_rpy=("
        << static_cast<double>(state.base_rpy[0]) << ","
        << static_cast<double>(state.base_rpy[1]) << ","
        << static_cast<double>(state.base_rpy[2]) << ")"
        << " qerr_mean=" << mean_abs_joint_error
        << " qerr_max=" << max_abs_joint_error
        << " paused=" << (viewer_paused_ ? "1" : "0");

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_inspector_;
        latest_viewer_inspector_text_ = oss.str();
        has_viewer_inspector_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

} // namespace mujoco_sim2sim
