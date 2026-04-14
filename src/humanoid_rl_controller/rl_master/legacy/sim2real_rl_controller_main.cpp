#include <atomic>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <memory>
#include <iostream>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <time.h>

#include "rl_master/RL_controller.h"
#include "rl_master/legacy/dds_robot_io.h"
#include "rl_master/robot_io.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/rl_protocol.h"
#include "rl_master/runtime/realtime_utils.h"

// Legacy standalone controller process.
// This path is kept only for compatibility with the Python interactive
// sim2sim backend and for isolated debugging of the old DDS-only topology.

static std::atomic<bool> g_run_flag(true);
static RL_controller *g_rl_controller = nullptr;
static RobotIO *g_robot_io = nullptr;

void handleSignal(int signal)
{
    if (signal != SIGINT)
    {
        return;
    }

    std::cout << "\nCtrl+C detected. Transitioning RL to safe hold..." << std::endl;
    g_run_flag = false;

    if (g_rl_controller)
    {
        g_rl_controller->estop();
    }
    if (g_robot_io)
    {
        g_robot_io->estop();
    }
}

void run_sim2real_rl_controller(RL_controller *controller, RobotIO *robot_io)
{
    if (!controller || !robot_io)
    {
        throw std::runtime_error("controller or robot_io is null");
    }

    g_rl_controller = controller;
    g_robot_io = robot_io;

    controller->RL_controller_Init();
    robot_io->connect();

    rl_master::runtime::RealtimeConfig runtime_rt = controller->runtimeCfg().realtime;
    (void)loadProcessRealtimeConfigFromYAML(RL_CFG_PATH, "controller", &runtime_rt);
    runtime_rt = rl_master::runtime::overrideRealtimeConfigFromEnv(runtime_rt, "RL_MASTER_CONTROLLER_RT_");
    rl_master::runtime::configureRealtime(runtime_rt, "RL_controller_legacy");

    rl_master::RobotStateData io_state;
    rl_master::TeleopCommand teleop_cmd;
    int mode_command = rl_master::kModeCodeMin;
    const auto warmup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!robot_io->read_state(io_state) && std::chrono::steady_clock::now() < warmup_deadline)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    (void)robot_io->read_control_command(teleop_cmd);
    mode_command = robot_io->read_mode_command(mode_command);

    const int control_hz = std::max(1, controller->runtimeCfg().RL_control_f);
    const long period_ns = std::max<long>(1, 1'000'000'000L / control_hz);
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    controller->start_time = std::chrono::high_resolution_clock::now();
    const auto phase_start = std::chrono::steady_clock::now();
    auto last_warn_time = std::chrono::steady_clock::now();

    std::cout << "------- start legacy RL controller --------" << std::endl;
    std::cout << "[RL_controller_legacy] frequency=" << control_hz << " Hz, period=" << period_ns << " ns" << std::endl;

    try
    {
        while (g_run_flag.load())
        {
            const auto loop_begin = std::chrono::steady_clock::now();

            if (!robot_io->read_state(io_state))
            {
                const auto now = std::chrono::steady_clock::now();
                if ((now - last_warn_time) > std::chrono::seconds(1))
                {
                    std::cerr << "[legacy] waiting robot state DDS stream..." << std::endl;
                    last_warn_time = now;
                }

                rl_master::RobotCommandData safe_cmd;
                safe_cmd.open_rl = rl_master::kOpenRlDisabled;
                (void)robot_io->write_command(safe_cmd);
                continue;
            }
            (void)robot_io->read_control_command(teleop_cmd);
            mode_command = robot_io->read_mode_command(mode_command);

            const double phase_t = std::chrono::duration_cast<std::chrono::duration<double>>(loop_begin - phase_start).count();
            const rl_master::RobotCommandData command = controller->step(io_state, teleop_cmd, mode_command, phase_t);

            if (!robot_io->write_command(command))
            {
                robot_io->estop();
                throw std::runtime_error("[legacy] failed to write command through RobotIO");
            }

            const auto loop_end = std::chrono::steady_clock::now();
            const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_begin).count();
            if (elapsed_us > controller->runtimeCfg().loop_overrun_warn_us &&
                (loop_end - last_warn_time) > std::chrono::seconds(1))
            {
                std::cerr << "[RL_controller_legacy] loop overrun: " << elapsed_us << " us" << std::endl;
                last_warn_time = loop_end;
            }

            next.tv_nsec += period_ns;
            if (next.tv_nsec >= 1'000'000'000)
            {
                next.tv_nsec -= 1'000'000'000;
                next.tv_sec++;
            }

            const int ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
            if (ret != 0 && ret != EINTR)
            {
                perror("clock_nanosleep");
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[legacy] exception in control loop: " << e.what() << std::endl;
    }

    controller->estop();
    robot_io->estop();
}

int main()
{
    signal(SIGINT, handleSignal);

    auto rl_controller = RL_controller::create();
    auto robot_io = std::make_unique<DdsRobotIO>();
    run_sim2real_rl_controller(rl_controller.get(), robot_io.get());

    return 0;
}
