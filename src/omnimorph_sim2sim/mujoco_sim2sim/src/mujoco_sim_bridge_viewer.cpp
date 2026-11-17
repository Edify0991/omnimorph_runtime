#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

namespace
{
std::string shellQuote(const std::string &value)
{
    std::string out = "'";
    for (const char c : value)
    {
        if (c == '\'')
        {
            out += "'\\''";
        }
        else
        {
            out += c;
        }
    }
    out += "'";
    return out;
}

std::string makeTimestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm tm_value{};
#if defined(_WIN32)
    localtime_s(&tm_value, &now);
#else
    localtime_r(&now, &tm_value);
#endif
    char buffer[32] = {};
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm_value);
    return std::string(buffer);
}

std::string sanitizeVideoFileName(std::string name)
{
    name = trimCopy(name);
    for (char &c : name)
    {
        const bool ok =
            std::isalnum(static_cast<unsigned char>(c)) != 0 ||
            c == '_' || c == '-' || c == '.';
        if (!ok)
        {
            c = '_';
        }
    }
    if (name.empty())
    {
        name = "mujoco_sim2sim_" + makeTimestamp() + ".mp4";
    }
    if (!endsWith(toLowerCopy(name), ".mp4"))
    {
        name += ".mp4";
    }
    return name;
}

std::array<double, 3> rotateByQuatWxyz(
    const std::array<double, 4> &q,
    const std::array<double, 3> &v)
{
    const std::array<double, 3> u{q[1], q[2], q[3]};
    const std::array<double, 3> uv{
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    };
    const std::array<double, 3> uuv{
        u[1] * uv[2] - u[2] * uv[1],
        u[2] * uv[0] - u[0] * uv[2],
        u[0] * uv[1] - u[1] * uv[0],
    };
    return {
        v[0] + 2.0 * (q[0] * uv[0] + uuv[0]),
        v[1] + 2.0 * (q[0] * uv[1] + uuv[1]),
        v[2] + 2.0 * (q[0] * uv[2] + uuv[2]),
    };
}

}

void MujocoSimBridge::initializeViewer()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_)
    {
        return;
    }

    viewer_state_ = std::make_unique<ViewerState>();
    if (!glfwInit())
    {
        RCLCPP_WARN(this->get_logger(), "GLFW init failed. Disable MuJoCo viewer and continue headless.");
        enable_viewer_ = false;
        viewer_state_.reset();
        return;
    }
    viewer_state_->glfw_initialized = true;

    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    viewer_state_->window = glfwCreateWindow(
        viewer_width_,
        viewer_height_,
        viewer_title_.c_str(),
        nullptr,
        nullptr);
    if (!viewer_state_->window)
    {
        RCLCPP_WARN(this->get_logger(), "GLFW window creation failed. Disable MuJoCo viewer and continue headless.");
        enable_viewer_ = false;
        shutdownViewer();
        return;
    }

    glfwMakeContextCurrent(viewer_state_->window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(viewer_state_->window, this);
    glfwSetMouseButtonCallback(
        viewer_state_->window,
        [](GLFWwindow *window, int button, int action, int mods) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerMouseButton(button, action, mods);
            }
        });
    glfwSetCursorPosCallback(
        viewer_state_->window,
        [](GLFWwindow *window, double xpos, double ypos) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerMouseMove(xpos, ypos);
            }
        });
    glfwSetScrollCallback(
        viewer_state_->window,
        [](GLFWwindow *window, double, double yoffset) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerScroll(yoffset);
            }
        });
    glfwSetKeyCallback(
        viewer_state_->window,
        [](GLFWwindow *window, int key, int, int action, int mods) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerKey(key, action, mods);
            }
        });

    mjv_makeScene(model_, &viewer_state_->scene, 4000);
    viewer_state_->scene_initialized = true;
    mjr_makeContext(model_, &viewer_state_->context, mjFONTSCALE_150);
    viewer_state_->context_initialized = true;
    viewer_state_->camera.type = mjCAMERA_FREE;
    viewer_state_->camera.azimuth = 90.0;
    viewer_state_->camera.elevation = -20.0;
    viewer_state_->camera.distance = 3.0;
    viewer_state_->last_render_time = std::chrono::steady_clock::now();
    viewer_state_->option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
    viewer_state_->option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo viewer enabled: %dx%d @ %.1fHz",
        viewer_width_,
        viewer_height_,
        viewer_fps_);
#else
    if (enable_viewer_)
    {
        RCLCPP_WARN(this->get_logger(), "Viewer requested but mujoco_sim2sim was built without GLFW support.");
        enable_viewer_ = false;
    }
#endif
}

void MujocoSimBridge::initializeVideoRecorder()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_video_recording_)
    {
        return;
    }
    if (!model_ || !data_)
    {
        RCLCPP_WARN(this->get_logger(), "Video recording requested before MuJoCo model/data are ready. Disable recording.");
        enable_video_recording_ = false;
        return;
    }

    std::filesystem::path output_dir(video_output_dir_);
    if (output_dir.is_relative())
    {
        output_dir = std::filesystem::absolute(output_dir);
    }
    std::error_code mkdir_ec;
    std::filesystem::create_directories(output_dir, mkdir_ec);
    if (mkdir_ec)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to create video output dir '%s': %s. Disable recording.",
            output_dir.string().c_str(),
            mkdir_ec.message().c_str());
        enable_video_recording_ = false;
        return;
    }
    const std::filesystem::path output_path = output_dir / sanitizeVideoFileName(video_output_name_);
    video_output_path_ = output_path.lexically_normal().string();

    auto state = std::make_unique<VideoRecorderState>();
    if ((!viewer_state_ || !viewer_state_->glfw_initialized) && !instinct_depth_camera_state_)
    {
        if (!glfwInit())
        {
            RCLCPP_WARN(this->get_logger(), "GLFW init failed. Disable native sim2sim video recording.");
            enable_video_recording_ = false;
            return;
        }
        state->owns_glfw = true;
    }

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    state->window = glfwCreateWindow(1, 1, "MuJoCo Sim2Sim Video Recorder", nullptr, nullptr);
    if (!state->window)
    {
        RCLCPP_WARN(this->get_logger(), "Hidden GLFW context creation failed. Disable video recording.");
        enable_video_recording_ = false;
        return;
    }
    glfwMakeContextCurrent(state->window);

    model_->vis.global.offwidth = video_width_;
    model_->vis.global.offheight = video_height_;
    mjv_makeScene(model_, &state->scene, 4000);
    state->scene_initialized = true;
    mjr_makeContext(model_, &state->context, mjFONTSCALE_150);
    state->context_initialized = true;
    state->data = mj_makeData(model_);
    if (!state->data)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to allocate MuJoCo data for video recording. Disable recording.");
        enable_video_recording_ = false;
        return;
    }
    state->camera.type = mjCAMERA_FREE;
    state->camera.azimuth = video_follow_azimuth_;
    state->camera.elevation = video_follow_elevation_;
    state->camera.distance = video_follow_distance_;
    state->option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
    state->option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;

    const std::string frame_size = std::to_string(video_width_) + "x" + std::to_string(video_height_);
    std::ostringstream command;
    command << shellQuote(video_ffmpeg_path_)
            << " -hide_banner -loglevel error -y"
            << " -f rawvideo -pix_fmt rgb24"
            << " -s " << frame_size
            << " -r " << std::fixed << std::setprecision(6) << video_fps_
            << " -i - -an"
            << " -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p"
            << " " << shellQuote(video_output_path_);

    state->pipe = popen(command.str().c_str(), "w");
    if (!state->pipe)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Failed to start ffmpeg for sim2sim video recording. Disable recording. command=%s",
            command.str().c_str());
        enable_video_recording_ = false;
        return;
    }

    const size_t frame_bytes = static_cast<size_t>(video_width_) * static_cast<size_t>(video_height_) * 3U;
    state->readback_rgb.assign(frame_bytes, 0U);
    state->frame_rgb.assign(frame_bytes, 0U);
    state->output_path = video_output_path_;
    next_video_frame_time_ = std::numeric_limits<double>::quiet_NaN();
    video_frame_count_ = 0;
    video_queue_overflow_warned_ = false;
    video_recorder_stop_requested_.store(false);
    video_recorder_failed_.store(false);
    {
        std::lock_guard<std::mutex> lock(video_recorder_mutex_);
        pending_video_frames_.clear();
    }
    video_recorder_state_ = std::move(state);
    video_recorder_thread_ = std::thread([this]() {
        while (true)
        {
            VideoFrameSnapshot snapshot;
            {
                std::unique_lock<std::mutex> lock(video_recorder_mutex_);
                video_recorder_cv_.wait(
                    lock,
                    [this]() {
                        return video_recorder_stop_requested_.load() ||
                               !pending_video_frames_.empty();
                    });

                if (pending_video_frames_.empty())
                {
                    if (video_recorder_stop_requested_.load())
                    {
                        break;
                    }
                    continue;
                }

                snapshot = std::move(pending_video_frames_.front());
                pending_video_frames_.pop_front();
            }

            if (!writeVideoFrame(snapshot))
            {
                video_recorder_failed_.store(true);
                break;
            }
        }
    });

    RCLCPP_INFO(
        this->get_logger(),
        "Native sim2sim video recording started: %s (%dx%d @ %.1f fps, async physical-time sampling)",
        video_output_path_.c_str(),
        video_width_,
        video_height_,
        video_fps_);
#else
    if (enable_video_recording_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Video recording requested but mujoco_sim2sim was built without GLFW/offscreen viewer support.");
        enable_video_recording_ = false;
    }
#endif
}

void MujocoSimBridge::initializeInstinctDepthCamera()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_instinct_depth_camera_)
    {
        return;
    }
    if (!model_ || !data_)
    {
        throw std::runtime_error("Instinct depth camera requested before MuJoCo model/data are ready");
    }

    const int output_width = instinct_depth_width_ - instinct_depth_crop_[2] - instinct_depth_crop_[3];
    const int output_height = instinct_depth_height_ - instinct_depth_crop_[0] - instinct_depth_crop_[1];
    if (output_width <= 0 || output_height <= 0)
    {
        throw std::runtime_error("Instinct depth crop removes the entire rendered image");
    }

    auto state = std::make_unique<InstinctDepthCameraState>();
    state->camera_id = mj_name2id(model_, mjOBJ_CAMERA, instinct_depth_camera_name_.c_str());
    state->parent_body_id = mj_name2id(model_, mjOBJ_BODY, instinct_depth_parent_body_name_.c_str());
    if (state->camera_id < 0)
    {
        throw std::runtime_error(
            "Instinct depth camera '" + instinct_depth_camera_name_ + "' is absent from MuJoCo XML");
    }
    if (state->parent_body_id < 0)
    {
        throw std::runtime_error(
            "Instinct depth parent body '" + instinct_depth_parent_body_name_ + "' is absent from MuJoCo XML");
    }

    if (!viewer_state_ || !viewer_state_->glfw_initialized)
    {
        if (!glfwInit())
        {
            throw std::runtime_error("GLFW init failed for the Instinct sim2sim depth renderer");
        }
        state->owns_glfw = true;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    state->window = glfwCreateWindow(1, 1, "MuJoCo Instinct Depth Camera", nullptr, nullptr);
    if (!state->window)
    {
        throw std::runtime_error("Hidden GLFW context creation failed for the Instinct sim2sim depth renderer");
    }
    glfwMakeContextCurrent(state->window);

    // The camera node only gives MuJoCo a named rendering handle.  Position
    // and orientation are updated from waist_yaw each frame to reproduce the
    // Isaac grouped raycaster's yaw-only attachment.
    model_->cam_bodyid[state->camera_id] = 0;
    model_->cam_mode[state->camera_id] = mjCAMLIGHT_FIXED;
    model_->cam_fovy[state->camera_id] = instinct_depth_vertical_fov_deg_;
    model_->vis.global.offwidth = instinct_depth_width_;
    model_->vis.global.offheight = instinct_depth_height_;
    mjv_makeScene(model_, &state->scene, 4000);
    state->scene_initialized = true;
    mjr_makeContext(model_, &state->context, mjFONTSCALE_150);
    state->context_initialized = true;
    mjr_setBuffer(mjFB_OFFSCREEN, &state->context);
    state->context.readDepthMap = mjDEPTH_ZEROFAR;
    state->camera.type = mjCAMERA_FIXED;
    state->camera.fixedcamid = state->camera_id;
    std::fill(std::begin(state->option.geomgroup), std::end(state->option.geomgroup), 0);
    state->option.geomgroup[0] = 1;

    const size_t raw_count = static_cast<size_t>(instinct_depth_width_) * static_cast<size_t>(instinct_depth_height_);
    const size_t processed_count = static_cast<size_t>(output_width) * static_cast<size_t>(output_height);
    state->raw_depth.assign(raw_count, 0.0f);
    state->processed_depth.assign(processed_count, 0.0f);
    state->blur_scratch.assign(processed_count, 0.0f);
    state->history.clear();
    state->next_frame_time = std::numeric_limits<double>::quiet_NaN();
    instinct_depth_camera_state_ = std::move(state);

    const size_t expected_dim = processed_count * static_cast<size_t>(instinct_depth_history_frames_);
    bool declared_feature = false;
    for (const auto &spec : controller_runtime_.runtimeCfg().external_observations)
    {
        if (spec.name == instinct_depth_feature_name_)
        {
            declared_feature = true;
            if (spec.dim > 0 && static_cast<size_t>(spec.dim) != expected_dim)
            {
                throw std::runtime_error(
                    "Instinct depth feature '" + instinct_depth_feature_name_ + "' declares dim " +
                    std::to_string(spec.dim) + ", but renderer produces " + std::to_string(expected_dim));
            }
        }
    }
    if (!declared_feature)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Instinct depth renderer is enabled but active profile declares no external observation '%s'. "
            "The data will be injected, but the Parkour adapter will not receive it until that profile declares it.",
            instinct_depth_feature_name_.c_str());
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Instinct synthetic depth camera ready: %s <- %s, source=%dx%d, crop=[%d,%d,%d,%d], history=%dx every %d frames, feature_dim=%zu",
        instinct_depth_camera_name_.c_str(),
        instinct_depth_parent_body_name_.c_str(),
        instinct_depth_width_,
        instinct_depth_height_,
        instinct_depth_crop_[0],
        instinct_depth_crop_[1],
        instinct_depth_crop_[2],
        instinct_depth_crop_[3],
        instinct_depth_history_frames_,
        instinct_depth_history_skip_,
        expected_dim);
#else
    if (enable_instinct_depth_camera_)
    {
        throw std::runtime_error(
            "Instinct sim2sim depth rendering requires GLFW/MuJoCo viewer support; rebuild with MUJOCO_SIM2SIM_ENABLE_VIEWER=ON");
    }
#endif
}

void MujocoSimBridge::shutdownInstinctDepthCamera()
{
    instinct_depth_camera_state_.reset();
}

void MujocoSimBridge::updateInstinctDepthCameraFeature()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_instinct_depth_camera_ || !instinct_depth_camera_state_ || !model_ || !data_)
    {
        return;
    }
    auto &state = *instinct_depth_camera_state_;
    const double sim_time = data_->time;
    if (!std::isfinite(state.next_frame_time))
    {
        state.next_frame_time = sim_time;
    }
    if (sim_time + 1.0e-9 < state.next_frame_time && !state.history.empty())
    {
        return;
    }
    const double period = 1.0 / instinct_depth_camera_hz_;
    while (sim_time + 1.0e-9 >= state.next_frame_time)
    {
        state.next_frame_time += period;
    }

    const double *parent_quat_ptr = data_->xquat + 4 * state.parent_body_id;
    const std::array<double, 4> parent_quat{
        parent_quat_ptr[0], parent_quat_ptr[1], parent_quat_ptr[2], parent_quat_ptr[3]};
    const double yaw = std::atan2(
        2.0 * (parent_quat[0] * parent_quat[3] + parent_quat[1] * parent_quat[2]),
        1.0 - 2.0 * (parent_quat[2] * parent_quat[2] + parent_quat[3] * parent_quat[3]));
    const std::array<double, 4> yaw_quat{
        std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
    // Isaac ray cameras look along +X / +Z up, while MuJoCo cameras look
    // along -Z / +Y up.  This is the same conversion used by instinct_jc's
    // reference sim2sim script.
    const std::array<double, 4> isaac_camera_from_mujoco{0.5, 0.5, -0.5, -0.5};
    const auto camera_offset_quat = normalizeQuatWxyz(
        multiplyQuatWxyz(instinct_depth_offset_rot_wxyz_, isaac_camera_from_mujoco));
    const auto offset_world = rotateByQuatWxyz(yaw_quat, instinct_depth_offset_pos_);
    const double *parent_pos = data_->xpos + 3 * state.parent_body_id;
    model_->cam_pos[3 * state.camera_id + 0] = parent_pos[0] + offset_world[0];
    model_->cam_pos[3 * state.camera_id + 1] = parent_pos[1] + offset_world[1];
    model_->cam_pos[3 * state.camera_id + 2] = parent_pos[2] + offset_world[2];
    const auto camera_quat = multiplyQuatWxyz(yaw_quat, camera_offset_quat);
    for (int i = 0; i < 4; ++i)
    {
        model_->cam_quat[4 * state.camera_id + i] = camera_quat[static_cast<size_t>(i)];
    }

    glfwMakeContextCurrent(state.window);
    mjr_setBuffer(mjFB_OFFSCREEN, &state.context);
    const mjrRect viewport{0, 0, instinct_depth_width_, instinct_depth_height_};
    const auto old_segment = state.scene.flags[mjRND_SEGMENT];
    const auto old_idcolor = state.scene.flags[mjRND_IDCOLOR];
    mjv_updateScene(
        model_, data_, &state.option, nullptr, &state.camera, mjCAT_ALL, &state.scene);
    state.scene.flags[mjRND_SEGMENT] = 1;
    state.scene.flags[mjRND_IDCOLOR] = 1;
    mjr_render(viewport, &state.scene, &state.context);
    mjr_readPixels(nullptr, state.raw_depth.data(), viewport, &state.context);
    state.scene.flags[mjRND_SEGMENT] = old_segment;
    state.scene.flags[mjRND_IDCOLOR] = old_idcolor;

    const float znear = static_cast<float>(model_->vis.map.znear * model_->stat.extent);
    const float zfar = static_cast<float>(model_->vis.map.zfar * model_->stat.extent);
    const float c = -0.5f * (-(zfar + znear) / (zfar - znear)) - 0.5f;
    const float d = -0.5f * (-(2.0f * zfar * znear) / (zfar - znear));
    const int top = instinct_depth_crop_[0];
    const int left = instinct_depth_crop_[2];
    const int output_width = instinct_depth_width_ - instinct_depth_crop_[2] - instinct_depth_crop_[3];
    const int output_height = instinct_depth_height_ - instinct_depth_crop_[0] - instinct_depth_crop_[1];
    for (int row = 0; row < output_height; ++row)
    {
        const int source_row = top + row;
        // mjr_readPixels is bottom-up.  Python's Renderer flips it before
        // crop/normalization, so mirror that order here.
        const int raw_row = instinct_depth_height_ - 1 - source_row;
        for (int col = 0; col < output_width; ++col)
        {
            const float raw = state.raw_depth[static_cast<size_t>(raw_row) * instinct_depth_width_ + left + col];
            float metric = d / (raw + c);
            if (!std::isfinite(metric))
            {
                metric = static_cast<float>(instinct_depth_range_max_);
            }
            state.processed_depth[static_cast<size_t>(row) * output_width + col] = metric;
        }
    }

    if (instinct_depth_blur_kernel_ > 1)
    {
        const int radius = instinct_depth_blur_kernel_ / 2;
        const double sigma = instinct_depth_blur_sigma_ > 0.0 ? instinct_depth_blur_sigma_ : 0.3 * (radius - 1) + 0.8;
        std::vector<double> kernel(static_cast<size_t>(instinct_depth_blur_kernel_), 0.0);
        double kernel_sum = 0.0;
        for (int k = -radius; k <= radius; ++k)
        {
            const double value = std::exp(-0.5 * static_cast<double>(k * k) / (sigma * sigma));
            kernel[static_cast<size_t>(k + radius)] = value;
            kernel_sum += value;
        }
        for (double &value : kernel)
        {
            value /= kernel_sum;
        }
        for (int row = 0; row < output_height; ++row)
        {
            for (int col = 0; col < output_width; ++col)
            {
                double sum = 0.0;
                for (int k = -radius; k <= radius; ++k)
                {
                    const int sample_col = std::clamp(col + k, 0, output_width - 1);
                    sum += kernel[static_cast<size_t>(k + radius)] *
                           state.processed_depth[static_cast<size_t>(row) * output_width + sample_col];
                }
                state.blur_scratch[static_cast<size_t>(row) * output_width + col] = static_cast<float>(sum);
            }
        }
        for (int row = 0; row < output_height; ++row)
        {
            for (int col = 0; col < output_width; ++col)
            {
                double sum = 0.0;
                for (int k = -radius; k <= radius; ++k)
                {
                    const int sample_row = std::clamp(row + k, 0, output_height - 1);
                    sum += kernel[static_cast<size_t>(k + radius)] *
                           state.blur_scratch[static_cast<size_t>(sample_row) * output_width + col];
                }
                state.processed_depth[static_cast<size_t>(row) * output_width + col] = static_cast<float>(sum);
            }
        }
    }
    for (float &value : state.processed_depth)
    {
        value = std::clamp(value, static_cast<float>(instinct_depth_range_min_), static_cast<float>(instinct_depth_range_max_));
        value = (value - static_cast<float>(instinct_depth_range_min_)) /
                static_cast<float>(instinct_depth_range_max_ - instinct_depth_range_min_);
    }

    const size_t required_history = static_cast<size_t>(
        1 + instinct_depth_history_skip_ * (instinct_depth_history_frames_ - 1));
    const size_t history_capacity = std::max<size_t>(
        static_cast<size_t>(std::ceil(instinct_depth_camera_hz_)), required_history);
    if (state.history.empty())
    {
        for (size_t i = 0; i < history_capacity; ++i)
        {
            state.history.push_back(state.processed_depth);
        }
    }
    else
    {
        state.history.push_back(state.processed_depth);
        while (state.history.size() > history_capacity)
        {
            state.history.pop_front();
        }
    }

    std::vector<float> flattened;
    flattened.reserve(state.processed_depth.size() * static_cast<size_t>(instinct_depth_history_frames_));
    const size_t first = state.history.size() - required_history;
    for (int frame = 0; frame < instinct_depth_history_frames_; ++frame)
    {
        const size_t history_index = first + static_cast<size_t>(frame * instinct_depth_history_skip_);
        const auto &image = state.history[history_index];
        flattened.insert(flattened.end(), image.begin(), image.end());
    }
    controller_runtime_.controller().setExternalObservationFeature(
        instinct_depth_feature_name_, flattened, rl_master::monotonicTimeSec());
#endif
}

void MujocoSimBridge::shutdownVideoRecorder()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!video_recorder_state_)
    {
        return;
    }
    video_recorder_stop_requested_.store(true);
    video_recorder_cv_.notify_all();
    if (video_recorder_thread_.joinable())
    {
        video_recorder_thread_.join();
    }
    int close_status = 0;
    if (video_recorder_state_->pipe)
    {
        close_status = pclose(video_recorder_state_->pipe);
        video_recorder_state_->pipe = nullptr;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Native sim2sim video saved: %s frames=%lu ffmpeg_status=%d%s",
        video_recorder_state_->output_path.c_str(),
        static_cast<unsigned long>(video_frame_count_),
        close_status,
        video_recorder_failed_.load() ? " recorder_failed=1" : "");
    {
        std::lock_guard<std::mutex> lock(video_recorder_mutex_);
        pending_video_frames_.clear();
    }
    video_recorder_state_.reset();
#endif
}

void MujocoSimBridge::recordVideoFrameIfDue()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_video_recording_ || !video_recorder_state_ || !data_ || video_recorder_failed_.load())
    {
        return;
    }
    const double sim_time = data_->time;
    if (!std::isfinite(next_video_frame_time_))
    {
        next_video_frame_time_ = sim_time;
    }
    const double period = 1.0 / std::max(1.0, video_fps_);
    if ((sim_time + 1.0e-9) < next_video_frame_time_)
    {
        return;
    }

    VideoFrameSnapshot snapshot;
    snapshot.qpos.assign(model_->nq, 0.0);
    snapshot.qvel.assign(model_->nv, 0.0);
    snapshot.ctrl.assign(model_->nu, 0.0);
    snapshot.sim_time = sim_time;
    for (int i = 0; i < model_->nq; ++i)
    {
        snapshot.qpos[i] = data_->qpos[i];
    }
    for (int i = 0; i < model_->nv; ++i)
    {
        snapshot.qvel[i] = data_->qvel[i];
    }
    for (int i = 0; i < model_->nu; ++i)
    {
        snapshot.ctrl[i] = data_->ctrl[i];
    }

    bool should_warn_overflow = false;
    while ((sim_time + 1.0e-9) >= next_video_frame_time_)
    {
        {
            std::lock_guard<std::mutex> lock(video_recorder_mutex_);
            constexpr size_t kMaxPendingVideoFrames = 3600;
            if (pending_video_frames_.size() >= kMaxPendingVideoFrames)
            {
                pending_video_frames_.pop_front();
                if (!video_queue_overflow_warned_)
                {
                    video_queue_overflow_warned_ = true;
                    should_warn_overflow = true;
                }
            }
            pending_video_frames_.push_back(snapshot);
        }
        next_video_frame_time_ += period;
    }
    video_recorder_cv_.notify_all();
    if (should_warn_overflow)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Native sim2sim video recorder queue exceeded capacity; dropping oldest frames to keep physics loop non-blocking.");
    }
#endif
}

bool MujocoSimBridge::writeVideoFrame(const VideoFrameSnapshot &snapshot)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!video_recorder_state_ || !video_recorder_state_->pipe || !video_recorder_state_->data)
    {
        return false;
    }
    auto &state = *video_recorder_state_;
    if (static_cast<int>(snapshot.qpos.size()) != model_->nq ||
        static_cast<int>(snapshot.qvel.size()) != model_->nv ||
        static_cast<int>(snapshot.ctrl.size()) != model_->nu)
    {
        return false;
    }
    for (int i = 0; i < model_->nq; ++i)
    {
        state.data->qpos[i] = snapshot.qpos[i];
    }
    for (int i = 0; i < model_->nv; ++i)
    {
        state.data->qvel[i] = snapshot.qvel[i];
    }
    for (int i = 0; i < model_->nu; ++i)
    {
        state.data->ctrl[i] = snapshot.ctrl[i];
    }
    state.data->time = snapshot.sim_time;
    mj_forward(model_, state.data);

    glfwMakeContextCurrent(state.window);
    mjr_setBuffer(mjFB_OFFSCREEN, &state.context);

    if (video_follow_robot_ && base_body_id_ >= 0 && base_body_id_ < model_->nbody)
    {
        state.camera.type = mjCAMERA_FREE;
        state.camera.lookat[0] = state.data->xpos[3 * base_body_id_ + 0] + video_follow_lookat_offset_[0];
        state.camera.lookat[1] = state.data->xpos[3 * base_body_id_ + 1] + video_follow_lookat_offset_[1];
        state.camera.lookat[2] = state.data->xpos[3 * base_body_id_ + 2] + video_follow_lookat_offset_[2];
        state.camera.distance = video_follow_distance_;
        state.camera.azimuth = video_follow_azimuth_;
        state.camera.elevation = video_follow_elevation_;
    }
    else if (viewer_state_)
    {
        state.camera = viewer_state_->camera;
    }

    state.option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
    state.option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;

    const mjrRect viewport{0, 0, video_width_, video_height_};
    mjv_updateScene(
        model_,
        state.data,
        &state.option,
        nullptr,
        &state.camera,
        mjCAT_ALL,
        &state.scene);
    appendComSupportOverlay(&state.scene, computeComSupportOverlay(state.data));
    mjr_render(viewport, &state.scene, &state.context);
    mjr_readPixels(state.readback_rgb.data(), nullptr, viewport, &state.context);

    const size_t row_bytes = static_cast<size_t>(video_width_) * 3U;
    for (int y = 0; y < video_height_; ++y)
    {
        const size_t src = static_cast<size_t>(video_height_ - 1 - y) * row_bytes;
        const size_t dst = static_cast<size_t>(y) * row_bytes;
        std::copy(
            state.readback_rgb.begin() + static_cast<std::ptrdiff_t>(src),
            state.readback_rgb.begin() + static_cast<std::ptrdiff_t>(src + row_bytes),
            state.frame_rgb.begin() + static_cast<std::ptrdiff_t>(dst));
    }

    const size_t expected = state.frame_rgb.size();
    const size_t written = fwrite(state.frame_rgb.data(), 1, expected, state.pipe);
    if (written != expected)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "ffmpeg pipe write failed for sim2sim video: wrote=%zu expected=%zu. Stop recording.",
            written,
            expected);
        return false;
    }
    ++video_frame_count_;
    return true;
#else
    (void)snapshot;
    return false;
#endif
}

void MujocoSimBridge::shutdownViewer()
{
    viewer_state_.reset();
}

void MujocoSimBridge::renderViewerFrame()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_ || !viewer_state_->window)
    {
        return;
    }
    if (glfwWindowShouldClose(viewer_state_->window))
    {
        RCLCPP_INFO(this->get_logger(), "MuJoCo viewer window closed by user. Continue headless.");
        enable_viewer_ = false;
        shutdownViewer();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double min_render_period = 1.0 / std::max(1.0, viewer_fps_);
    if (viewer_state_->last_render_time.time_since_epoch().count() != 0)
    {
        const double dt = std::chrono::duration<double>(now - viewer_state_->last_render_time).count();
        if (dt < min_render_period)
        {
            return;
        }
    }

    glfwMakeContextCurrent(viewer_state_->window);
    glfwPollEvents();

    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(viewer_state_->window, &fb_w, &fb_h);
    if (fb_w <= 0 || fb_h <= 0)
    {
        return;
    }

    const mjrRect viewport{0, 0, fb_w, fb_h};
    mjv_updateScene(
        model_,
        data_,
        &viewer_state_->option,
        nullptr,
        &viewer_state_->camera,
        mjCAT_ALL,
        &viewer_state_->scene);
    appendComSupportOverlay(&viewer_state_->scene, computeComSupportOverlay(data_));
    mjr_render(viewport, &viewer_state_->scene, &viewer_state_->context);

    if (viewer_show_hud_)
    {
        std::ostringstream left;
        std::ostringstream right;
        left << "Space: pause/resume\n"
             << "Right: step once\n"
             << "[ / ]: speed -/+\n"
             << "C: toggle contacts\n"
             << "B: toggle base omega\n"
             << "H: toggle HUD\n"
             << "Ncon";
        right << (viewer_paused_ ? "paused" : "running") << "\n"
              << "step\n"
              << sim_speed_scale_ << "x\n"
              << (viewer_show_contact_ ? "on" : "off") << "\n"
              << (viewer_show_base_speed_ ? "on" : "off") << "\n"
              << "on\n"
              << data_->ncon;

        if (viewer_show_base_speed_)
        {
            double wx = 0.0;
            double wy = 0.0;
            double wz = 0.0;
            if (base_free_qvel_adr_ >= 0 && (base_free_qvel_adr_ + 5) < model_->nv)
            {
                wx = data_->qvel[base_free_qvel_adr_ + 3];
                wy = data_->qvel[base_free_qvel_adr_ + 4];
                wz = data_->qvel[base_free_qvel_adr_ + 5];
            }
            left << "\nBase omega";
            right << "\n[" << wx << ", " << wy << ", " << wz << "]";
        }

        mjr_overlay(
            mjFONT_NORMAL,
            mjGRID_TOPLEFT,
            viewport,
            left.str().c_str(),
            right.str().c_str(),
            &viewer_state_->context);
    }

    glfwSwapBuffers(viewer_state_->window);
    viewer_state_->last_render_time = now;
#endif
}

void MujocoSimBridge::handleViewerMouseButton(int button, int action, int)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            viewer_mouse_left_down_ = true;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            viewer_mouse_middle_down_ = true;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            viewer_mouse_right_down_ = true;
        }
    }
    else if (action == GLFW_RELEASE)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            viewer_mouse_left_down_ = false;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            viewer_mouse_middle_down_ = false;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            viewer_mouse_right_down_ = false;
        }
    }
#else
    (void)button;
    (void)action;
#endif
}

void MujocoSimBridge::handleViewerMouseMove(double xpos, double ypos)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_ || !viewer_state_->window)
    {
        return;
    }

    const double dx = xpos - viewer_last_mouse_x_;
    const double dy = ypos - viewer_last_mouse_y_;
    viewer_last_mouse_x_ = xpos;
    viewer_last_mouse_y_ = ypos;

    if (!viewer_mouse_left_down_ && !viewer_mouse_middle_down_ && !viewer_mouse_right_down_)
    {
        return;
    }

    const int shift_pressed =
        (glfwGetKey(viewer_state_->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
        (glfwGetKey(viewer_state_->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    int action = mjMOUSE_ZOOM;
    if (viewer_mouse_right_down_)
    {
        action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (viewer_mouse_left_down_)
    {
        action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else if (viewer_mouse_middle_down_)
    {
        action = mjMOUSE_ZOOM;
    }

    int width = 0;
    int height = 0;
    glfwGetWindowSize(viewer_state_->window, &width, &height);
    const double norm = std::max(1, height);
    mjv_moveCamera(
        model_,
        action,
        dx / norm,
        dy / norm,
        &viewer_state_->scene,
        &viewer_state_->camera);
#else
    (void)xpos;
    (void)ypos;
#endif
}

void MujocoSimBridge::handleViewerScroll(double yoffset)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_)
    {
        return;
    }
    mjv_moveCamera(
        model_,
        mjMOUSE_ZOOM,
        0.0,
        -0.05 * yoffset,
        &viewer_state_->scene,
        &viewer_state_->camera);
#else
    (void)yoffset;
#endif
}

void MujocoSimBridge::handleViewerKey(int key, int action, int)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (action != GLFW_PRESS || !viewer_state_)
    {
        return;
    }

    if (key == GLFW_KEY_SPACE)
    {
        viewer_paused_ = !viewer_paused_;
        return;
    }
    if (key == GLFW_KEY_RIGHT)
    {
        viewer_step_once_ = true;
        viewer_paused_ = true;
        return;
    }
    if (key == GLFW_KEY_LEFT_BRACKET)
    {
        sim_speed_scale_ = std::max(0.1, sim_speed_scale_ / 1.25);
        return;
    }
    if (key == GLFW_KEY_RIGHT_BRACKET)
    {
        sim_speed_scale_ = std::min(4.0, sim_speed_scale_ * 1.25);
        return;
    }
    if (key == GLFW_KEY_C)
    {
        viewer_show_contact_ = !viewer_show_contact_;
        viewer_state_->option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
        viewer_state_->option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;
        return;
    }
    if (key == GLFW_KEY_B)
    {
        viewer_show_base_speed_ = !viewer_show_base_speed_;
        return;
    }
    if (key == GLFW_KEY_H)
    {
        viewer_show_hud_ = !viewer_show_hud_;
        return;
    }
#else
    (void)key;
    (void)action;
#endif
}

} // namespace mujoco_sim2sim
