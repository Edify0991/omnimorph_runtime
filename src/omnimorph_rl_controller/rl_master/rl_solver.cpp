#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "rl_master/mode_profile_registry.h"
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
                throw std::runtime_error(
                    std::string("[RL_solver] invalid --mode-id value: ") + argv[i + 1]);
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
                throw std::runtime_error(
                    std::string("[RL_solver] invalid --mode-id value: ") + arg);
            }
        }
    }
    return mode_id;
}
} // namespace

int main(int argc, char **argv)
{
    try
    {
        const int startup_mode_id = parseStartupModeId(argc, argv);

        std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry;
        try
        {
            mode_registry = rl_master::ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk");
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to load mode profile registry: " << e.what() << std::endl;
            return -1;
        }

        const auto &startup_spec = mode_registry->specForMode(startup_mode_id, false);
        const auto &startup_cfg = mode_registry->cfgForMode(startup_mode_id, false);
        const std::string startup_section = startup_spec.config_section;
        std::cout << "[RL_solver] startup mode_id=" << startup_mode_id
                  << ", config section=" << startup_section << std::endl;
        rl_master::runtime::RealtimeConfig runtime_rt = startup_cfg.realtime;
        (void)loadProcessRealtimeConfigFromYAML(RL_CFG_PATH, "solver", &runtime_rt);
        runtime_rt = rl_master::runtime::overrideRealtimeConfigFromEnv(runtime_rt, "RL_MASTER_SOLVER_RT_");
        rl_master::runtime::configureRealtime(runtime_rt, "RL_solver");

        std::unique_ptr<rl_master::solver::RobotSolver> solver =
            rl_master::solver::RobotSolver::create(startup_mode_id, mode_registry);
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
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] startup failed: " << e.what() << std::endl;
        return -1;
    }
}
