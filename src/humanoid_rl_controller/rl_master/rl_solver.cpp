#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rl_master/rl_cfg.h"
#include "rl_master/runtime/realtime_utils.h"
#include "rl_master/solver/robot_solver.h"

namespace
{
volatile std::sig_atomic_t g_stop_requested = 0;

void handleSignal(int signal_number)
{
    if (signal_number == SIGINT)
    {
        g_stop_requested = 1;
    }
}

int parseStartupModeId(int argc, char **argv)
{
    int mode_id = readDeployModeIdFromEnv("RL_MASTER_DEPLOY_MODE_ID", 0);
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i] ? argv[i] : "";
        if (arg == "--mode-id" && (i + 1) < argc)
        {
            try
            {
                mode_id = std::stoi(argv[i + 1]);
            }
            catch (const std::exception &)
            {
                std::cerr << "[RL_solver] invalid --mode-id value: " << argv[i + 1]
                          << ", fallback to " << mode_id << std::endl;
            }
            ++i;
            continue;
        }
        const std::string prefix = "--mode-id=";
        if (arg.rfind(prefix, 0) == 0)
        {
            try
            {
                mode_id = std::stoi(arg.substr(prefix.size()));
            }
            catch (const std::exception &)
            {
                std::cerr << "[RL_solver] invalid --mode-id value: " << arg
                          << ", fallback to " << mode_id << std::endl;
            }
        }
    }
    return mode_id;
}
} // namespace

int main(int argc, char **argv)
{
    const int startup_mode_id = parseStartupModeId(argc, argv);

    Sim2realCfg startup_cfg;
    const std::string startup_section = resolveDeployConfigSectionForModeFromYAML(
        RL_CFG_PATH,
        startup_mode_id,
        "engineai_walk");
    if (!startup_cfg.loadFromYAML(RL_CFG_PATH, startup_section))
    {
        std::cerr << "Failed to load startup config section '" << startup_section
                  << "' for realtime setup." << std::endl;
        return -1;
    }
    std::cout << "[RL_solver] startup mode_id=" << startup_mode_id
              << ", config section=" << startup_section << std::endl;
    rl_master::runtime::RealtimeConfig runtime_rt = startup_cfg.realtime;
    (void)loadProcessRealtimeConfigFromYAML(RL_CFG_PATH, "solver", &runtime_rt);
    runtime_rt = rl_master::runtime::overrideRealtimeConfigFromEnv(runtime_rt, "RL_MASTER_SOLVER_RT_");
    rl_master::runtime::configureRealtime(runtime_rt, "RL_solver");

    std::unique_ptr<rl_master::solver::RobotSolver> solver =
        rl_master::solver::RobotSolver::create(startup_mode_id);
    if (!solver)
    {
        std::cerr << "Failed to create RobotSolver." << std::endl;
        return -1;
    }

    if (!solver->initialize())
    {
        std::cerr << "Failed to initialize RobotSolver." << std::endl;
        return -1;
    }

    std::signal(SIGINT, handleSignal);

    std::atomic<bool> solver_finished{false};
    std::thread solver_thread([&solver, &solver_finished]() {
        solver->run();
        solver_finished.store(true);
    });

    while (!solver_finished.load())
    {
        if (g_stop_requested != 0)
        {
            std::cout << "\nCtrl+C detected. Requesting solver stop..." << std::endl;
            solver->requestStop();
            g_stop_requested = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (solver_thread.joinable())
    {
        solver_thread.join();
    }
    return 0;
}
