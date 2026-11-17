#pragma once

namespace mujoco_sim2sim
{

#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
struct MujocoSimBridge::ViewerState
{
    GLFWwindow *window = nullptr;
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    std::chrono::steady_clock::time_point last_render_time{};
    bool glfw_initialized = false;
    bool scene_initialized = false;
    bool context_initialized = false;

    ViewerState()
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
    }

    ~ViewerState()
    {
        if (context_initialized)
        {
            mjr_freeContext(&context);
        }
        if (scene_initialized)
        {
            mjv_freeScene(&scene);
        }
        if (window)
        {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        if (glfw_initialized)
        {
            glfwTerminate();
        }
    }
};
#else
struct MujocoSimBridge::ViewerState
{
};
#endif

#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
struct MujocoSimBridge::VideoRecorderState
{
    GLFWwindow *window = nullptr;
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    mjData *data = nullptr;
    FILE *pipe = nullptr;
    std::vector<unsigned char> readback_rgb;
    std::vector<unsigned char> frame_rgb;
    std::string output_path;
    bool owns_glfw = false;
    bool scene_initialized = false;
    bool context_initialized = false;

    VideoRecorderState()
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
    }

    ~VideoRecorderState()
    {
        if (pipe)
        {
            pclose(pipe);
            pipe = nullptr;
        }
        if (context_initialized)
        {
            mjr_freeContext(&context);
        }
        if (data)
        {
            mj_deleteData(data);
            data = nullptr;
        }
        if (scene_initialized)
        {
            mjv_freeScene(&scene);
        }
        if (window)
        {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        if (owns_glfw)
        {
            glfwTerminate();
        }
    }
};

struct MujocoSimBridge::InstinctDepthCameraState
{
    GLFWwindow *window = nullptr;
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    int camera_id = -1;
    int parent_body_id = -1;
    bool owns_glfw = false;
    bool scene_initialized = false;
    bool context_initialized = false;
    double next_frame_time = std::numeric_limits<double>::quiet_NaN();
    std::deque<std::vector<float>> history;
    std::vector<float> raw_depth;
    std::vector<float> processed_depth;
    std::vector<float> blur_scratch;

    InstinctDepthCameraState()
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
    }

    ~InstinctDepthCameraState()
    {
        if (context_initialized)
        {
            mjr_freeContext(&context);
        }
        if (scene_initialized)
        {
            mjv_freeScene(&scene);
        }
        if (window)
        {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        if (owns_glfw)
        {
            glfwTerminate();
        }
    }
};
#else
struct MujocoSimBridge::VideoRecorderState
{
};
struct MujocoSimBridge::InstinctDepthCameraState
{
};
#endif

} // namespace mujoco_sim2sim
