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
    if (!viewer_state_ || !viewer_state_->glfw_initialized)
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
    video_recorder_state_ = std::move(state);

    RCLCPP_INFO(
        this->get_logger(),
        "Native sim2sim video recording started: %s (%dx%d @ %.1f fps, physical-time sampling)",
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

void MujocoSimBridge::shutdownVideoRecorder()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!video_recorder_state_)
    {
        return;
    }
    int close_status = 0;
    if (video_recorder_state_->pipe)
    {
        close_status = pclose(video_recorder_state_->pipe);
        video_recorder_state_->pipe = nullptr;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Native sim2sim video saved: %s frames=%lu ffmpeg_status=%d",
        video_recorder_state_->output_path.c_str(),
        static_cast<unsigned long>(video_frame_count_),
        close_status);
    video_recorder_state_.reset();
#endif
}

void MujocoSimBridge::recordVideoFrameIfDue()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_video_recording_ || !video_recorder_state_ || !data_)
    {
        return;
    }
    const double sim_time = data_->time;
    if (!std::isfinite(next_video_frame_time_))
    {
        next_video_frame_time_ = sim_time;
    }
    const double period = 1.0 / std::max(1.0, video_fps_);
    while ((sim_time + 1.0e-9) >= next_video_frame_time_)
    {
        writeVideoFrame();
        if (!enable_video_recording_ || !video_recorder_state_)
        {
            break;
        }
        next_video_frame_time_ += period;
    }
#endif
}

void MujocoSimBridge::writeVideoFrame()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!video_recorder_state_ || !video_recorder_state_->pipe)
    {
        return;
    }
    auto &state = *video_recorder_state_;
    glfwMakeContextCurrent(state.window);
    mjr_setBuffer(mjFB_OFFSCREEN, &state.context);

    if (video_follow_robot_ && base_body_id_ >= 0 && base_body_id_ < model_->nbody)
    {
        state.camera.type = mjCAMERA_FREE;
        state.camera.lookat[0] = data_->xpos[3 * base_body_id_ + 0] + video_follow_lookat_offset_[0];
        state.camera.lookat[1] = data_->xpos[3 * base_body_id_ + 1] + video_follow_lookat_offset_[1];
        state.camera.lookat[2] = data_->xpos[3 * base_body_id_ + 2] + video_follow_lookat_offset_[2];
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
        data_,
        &state.option,
        nullptr,
        &state.camera,
        mjCAT_ALL,
        &state.scene);
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
        enable_video_recording_ = false;
        shutdownVideoRecorder();
        return;
    }
    ++video_frame_count_;
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
