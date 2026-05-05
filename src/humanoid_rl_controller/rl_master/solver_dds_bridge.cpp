#include "rl_master/solver_dds_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include "rl_master/KinConv.h"
#include "rl_master/runtime/realtime_utils.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;

std::string normalizeToken(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

int axisComponentIndex(const std::string &token_raw)
{
    const std::string token = normalizeToken(token_raw);
    if (token == "x" || token == "wx" || token == "roll")
    {
        return 0;
    }
    if (token == "y" || token == "wy" || token == "pitch")
    {
        return 1;
    }
    if (token == "z" || token == "wz" || token == "yaw")
    {
        return 2;
    }
    return -1;
}

std::array<float, 3> reorderVector3(
    const std::array<float, 3> &raw,
    const std::vector<std::string> &order,
    const std::vector<std::string> &expected_tokens)
{
    std::array<float, 3> out{0.0f, 0.0f, 0.0f};
    for (size_t raw_idx = 0; raw_idx < raw.size() && raw_idx < order.size(); ++raw_idx)
    {
        const int canonical_idx = axisComponentIndex(order[raw_idx]);
        if (canonical_idx >= 0 && canonical_idx < 3)
        {
            out[static_cast<size_t>(canonical_idx)] = raw[raw_idx];
        }
    }
    (void)expected_tokens;
    return out;
}

std::array<float, 4> quatMultiply(const std::array<float, 4> &lhs, const std::array<float, 4> &rhs)
{
    const float x1 = lhs[0], y1 = lhs[1], z1 = lhs[2], w1 = lhs[3];
    const float x2 = rhs[0], y2 = rhs[1], z2 = rhs[2], w2 = rhs[3];
    return {
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2};
}

std::array<float, 3> rotateVectorByQuat(
    const std::array<float, 3> &vec,
    const std::array<float, 4> &quat_xyzw)
{
    const std::vector<float> qv = {quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]};
    const float x = qv[0];
    const float y = qv[1];
    const float z = qv[2];
    const float w = qv[3];
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;
    return {
        (1.0f - 2.0f * (yy + zz)) * vec[0] + 2.0f * (xy - wz) * vec[1] + 2.0f * (xz + wy) * vec[2],
        2.0f * (xy + wz) * vec[0] + (1.0f - 2.0f * (xx + zz)) * vec[1] + 2.0f * (yz - wx) * vec[2],
        2.0f * (xz - wy) * vec[0] + 2.0f * (yz + wx) * vec[1] + (1.0f - 2.0f * (xx + yy)) * vec[2]};
}

std::array<float, 4> parseQuaternionFromImuMsg(
    const sensor_msgs::msg::Imu::SharedPtr &msg,
    const SourceContractImuInput &contract)
{
    const std::string quat_order = normalizeToken(contract.quat_order);
    if (quat_order == "wxyz")
    {
        return {
            static_cast<float>(msg->orientation.y),
            static_cast<float>(msg->orientation.z),
            static_cast<float>(msg->orientation.w),
            static_cast<float>(msg->orientation.x)};
    }
    return {
        static_cast<float>(msg->orientation.x),
        static_cast<float>(msg->orientation.y),
        static_cast<float>(msg->orientation.z),
        static_cast<float>(msg->orientation.w)};
}

std::array<float, 4> parseQuaternionFromOdomMsg(const nav_msgs::msg::Odometry::SharedPtr &msg)
{
    return {
        static_cast<float>(msg->pose.pose.orientation.x),
        static_cast<float>(msg->pose.pose.orientation.y),
        static_cast<float>(msg->pose.pose.orientation.z),
        static_cast<float>(msg->pose.pose.orientation.w)};
}
}

SolverDdsBridge::~SolverDdsBridge()
{
    disconnect();
}

void SolverDdsBridge::connect()
{
    connect(StateTelemetryConfig{});
}

void SolverDdsBridge::connect(const StateTelemetryConfig &telemetry_config)
{
    disconnect();

    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("rl_solver_dds_bridge");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    state_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    teleop_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        rl_master::dds::kTopicTeleopCommand,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            rl_master::TeleopCommand cmd;
            cmd.vx = static_cast<float>(msg->linear.x);
            cmd.vy = static_cast<float>(msg->linear.y);
            cmd.dyaw = static_cast<float>(msg->angular.z);
            std::lock_guard<std::mutex> lock(teleop_mutex_);
            latest_teleop_ = cmd;
            has_teleop_ = true;
        });

    mode_control_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicModeControl,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(mode_control_mutex_);
            latest_mode_control_word_ = msg->data;
            has_mode_control_word_ = true;
        });

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/yesense",
        rclcpp::QoS(rclcpp::KeepLast(30)).best_effort(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            SourceContract contract_snapshot;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                contract_snapshot = source_contract_;
            }

            const auto &imu_contract = contract_snapshot.imu_input;
            std::array<float, 3> raw_ang_vel{
                static_cast<float>(msg->angular_velocity.x),
                static_cast<float>(msg->angular_velocity.y),
                static_cast<float>(msg->angular_velocity.z)};
            std::array<float, 3> raw_lin_acc{
                static_cast<float>(msg->linear_acceleration.x),
                static_cast<float>(msg->linear_acceleration.y),
                static_cast<float>(msg->linear_acceleration.z)};
            std::array<float, 3> canonical_ang_vel = reorderVector3(
                raw_ang_vel,
                imu_contract.ang_vel_order,
                {"x", "y", "z"});
            std::array<float, 3> canonical_lin_acc = raw_lin_acc;

            std::array<float, 4> canonical_quat{0.0f, 0.0f, 0.0f, 1.0f};
            const std::string payload = normalizeToken(imu_contract.payload);
            if (payload == "quaternion")
            {
                canonical_quat = parseQuaternionFromImuMsg(msg, imu_contract);
            }
            else
            {
                std::array<float, 3> raw_euler{
                    static_cast<float>(msg->orientation.x),
                    static_cast<float>(msg->orientation.y),
                    static_cast<float>(msg->orientation.z)};
                std::array<float, 3> canonical_rpy = reorderVector3(
                    raw_euler,
                    imu_contract.euler_order,
                    {"roll", "pitch", "yaw"});
                if (normalizeToken(imu_contract.euler_unit) == "deg")
                {
                    constexpr float kDegToRad = kPi / 180.0f;
                    for (float &value : canonical_rpy)
                    {
                        value *= kDegToRad;
                    }
                }
                canonical_quat = rpyToQuat(canonical_rpy[0], canonical_rpy[1], canonical_rpy[2]);
            }

            std::array<float, 4> alignment_quat = rpyToQuat(
                imu_contract.frame_alignment_rpy.size() > 0 ? imu_contract.frame_alignment_rpy[0] : 0.0f,
                imu_contract.frame_alignment_rpy.size() > 1 ? imu_contract.frame_alignment_rpy[1] : 0.0f,
                imu_contract.frame_alignment_rpy.size() > 2 ? imu_contract.frame_alignment_rpy[2] : 0.0f);
            canonical_quat = quatMultiply(canonical_quat, alignment_quat);
            canonical_ang_vel = rotateVectorByQuat(canonical_ang_vel, alignment_quat);
            canonical_lin_acc = rotateVectorByQuat(canonical_lin_acc, alignment_quat);

            const std::vector<float> rpy_vec = quaternion_to_euler_array({
                canonical_quat[0], canonical_quat[1], canonical_quat[2], canonical_quat[3]});
            std::array<float, 3> canonical_rpy{
                rpy_vec.size() > 0 ? rpy_vec[0] : 0.0f,
                rpy_vec.size() > 1 ? rpy_vec[1] : 0.0f,
                rpy_vec.size() > 2 ? rpy_vec[2] : 0.0f};

            std::function<void(
                const std::array<float, 3> &,
                const std::array<float, 4> &,
                const std::array<float, 3> &,
                double)> callback;
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                imu_ang_vel_ = canonical_ang_vel;
                imu_lin_acc_ = canonical_lin_acc;
                imu_quat_ = canonical_quat;
                imu_rpy_ = canonical_rpy;
                has_imu_sample_ = true;
                callback = imu_sample_callback_;
            }

            if (callback)
            {
                callback(
                    canonical_ang_vel,
                    canonical_quat,
                    canonical_rpy,
                    rl_master::monotonicTimeSec());
            }
        });

    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_config_ = telemetry_config;
        has_mirrored_state_ = false;
    }
    stop_requested_.store(false);

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executorLoop(); });
    telemetry_thread_ = std::thread([this]() { telemetryLoop(); });
}

void SolverDdsBridge::disconnect()
{
    stop_requested_.store(true);
    telemetry_cv_.notify_all();

    if (executor_)
    {
        executor_->cancel();
    }

    if (telemetry_thread_.joinable())
    {
        telemetry_thread_.join();
    }
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }

    if (executor_ && node_)
    {
        executor_->remove_node(node_);
    }

    imu_sub_.reset();
    odom_sub_.reset();
    mode_control_sub_.reset();
    teleop_sub_.reset();
    state_pub_.reset();
    executor_.reset();
    node_.reset();
}

void SolverDdsBridge::updateStateTelemetryConfig(const StateTelemetryConfig &telemetry_config)
{
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_config_ = telemetry_config;
    }
    telemetry_cv_.notify_all();
}

void SolverDdsBridge::updateSourceContract(const SourceContract &source_contract)
{
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        source_contract_ = source_contract;
    }
    configureOdomSubscription();
}

void SolverDdsBridge::configureOdomSubscription()
{
    if (!node_)
    {
        return;
    }

    SourceContract contract_snapshot;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        contract_snapshot = source_contract_;
    }

    const auto &cfg = contract_snapshot.base_velocity_estimator;
    if (!cfg.enabled || !cfg.use_odom_velocity_measurement || cfg.odom_topic.empty())
    {
        odom_sub_.reset();
        active_odom_topic_.clear();
        std::lock_guard<std::mutex> lock(imu_mutex_);
        has_odom_lin_vel_ = false;
        return;
    }
    if (odom_sub_ && active_odom_topic_ == cfg.odom_topic)
    {
        return;
    }

    odom_sub_.reset();
    active_odom_topic_ = cfg.odom_topic;
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        cfg.odom_topic,
        rclcpp::QoS(rclcpp::KeepLast(30)).best_effort(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            SourceContract contract_snapshot;
            std::array<float, 4> latest_imu_quat{0.0f, 0.0f, 0.0f, 1.0f};
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                contract_snapshot = source_contract_;
                latest_imu_quat = imu_quat_;
            }

            std::array<float, 3> velocity{
                static_cast<float>(msg->twist.twist.linear.x),
                static_cast<float>(msg->twist.twist.linear.y),
                static_cast<float>(msg->twist.twist.linear.z)};

            const std::string frame = normalizeToken(contract_snapshot.base_velocity_estimator.odom_velocity_frame);
            if (frame == "body")
            {
                std::array<float, 4> q = parseQuaternionFromOdomMsg(msg);
                const float q_norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
                if (q_norm < 1.0e-4f)
                {
                    q = latest_imu_quat;
                }
                velocity = rotateVectorByQuat(velocity, q);
            }

            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                odom_lin_vel_w_ = velocity;
                has_odom_lin_vel_ = true;
            }
        });
}

void SolverDdsBridge::setImuSampleCallback(
    std::function<void(
        const std::array<float, 3> &ang_vel,
        const std::array<float, 4> &quat,
        const std::array<float, 3> &rpy,
        double monotonic_time_sec)> callback)
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_sample_callback_ = std::move(callback);
}

void SolverDdsBridge::executorLoop()
{
    try
    {
        if (executor_)
        {
            executor_->spin();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[SolverDdsBridge] executor thread exception: " << e.what() << std::endl;
    }
}

void SolverDdsBridge::telemetryLoop()
{
    while (!stop_requested_.load())
    {
        rl_master::RobotStateData state;
        double publish_hz = 0.0;
        bool enabled = false;
        bool has_state = false;

        {
            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            enabled = telemetry_config_.enabled;
            publish_hz = telemetry_config_.publish_hz;
            has_state = has_mirrored_state_;

            if (!enabled || publish_hz <= 0.0 || !has_state)
            {
                telemetry_cv_.wait_for(
                    lock,
                    std::chrono::milliseconds(100),
                    [this]() {
                        return stop_requested_.load() ||
                               (telemetry_config_.enabled && telemetry_config_.publish_hz > 0.0 && has_mirrored_state_);
                    });
                continue;
            }

            state = latest_mirrored_state_;
        }

        if (state_pub_)
        {
            state_pub_->publish(rl_master::dds::encodeRobotState(state));
        }

        std::unique_lock<std::mutex> lock(telemetry_mutex_);
        telemetry_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / publish_hz)),
            [this]() { return stop_requested_.load(); });
    }
}

bool SolverDdsBridge::readLatestTeleopCommand(rl_master::TeleopCommand *command)
{
    if (!command)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    if (!has_teleop_)
    {
        return false;
    }
    *command = latest_teleop_;
    return true;
}

bool SolverDdsBridge::readLatestModeControlWord(int *control_word)
{
    if (!control_word)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(mode_control_mutex_);
    if (!has_mode_control_word_)
    {
        return false;
    }
    *control_word = latest_mode_control_word_;
    return true;
}

void SolverDdsBridge::buildRobotStateData(
    const std::vector<JointData> &joint_state,
    rl_master::RobotStateData *state)
{
    if (!state)
    {
        return;
    }

    *state = rl_master::RobotStateData{};
    const size_t n = joint_state.size();
    state->protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    state->active_joint_count = static_cast<int>(n);
    state->joint_q.assign(n, 0.0f);
    state->joint_dq.assign(n, 0.0f);
    state->joint_tau.assign(n, 0.0f);
    for (size_t i = 0; i < n; ++i)
    {
        state->joint_q[i] = joint_state[i].q;
        state->joint_dq[i] = joint_state[i].dq;
        state->joint_tau[i] = joint_state[i].tau;
    }

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        state->base_ang_vel = imu_ang_vel_;
        state->base_lin_acc = imu_lin_acc_;
        state->base_quat = imu_quat_;
        state->base_rpy = imu_rpy_;
        state->base_lin_acc_valid = has_imu_sample_;
        if (has_odom_lin_vel_)
        {
            state->base_lin_vel = odom_lin_vel_w_;
            state->base_lin_vel_valid = true;
        }
    }
}

void SolverDdsBridge::mirrorRobotState(const std::vector<JointData> &joint_state)
{
    rl_master::RobotStateData state;
    buildRobotStateData(joint_state, &state);
    mirrorRobotState(state);
}

void SolverDdsBridge::mirrorRobotState(const rl_master::RobotStateData &state)
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

std::array<float, 4> SolverDdsBridge::rpyToQuat(float roll, float pitch, float yaw)
{
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);

    std::array<float, 4> q{};
    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    return q;
}
