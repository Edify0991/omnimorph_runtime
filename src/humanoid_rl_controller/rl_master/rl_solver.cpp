#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <thread>

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
} // namespace

int main()
{
    rl_master::runtime::setRealtimePriority(2, 90);

    std::unique_ptr<rl_master::solver::RobotSolver> solver = rl_master::solver::RobotSolver::create();
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
