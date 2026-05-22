#include <exception>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "joint_motor_test/joint_motor_test_runner.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        joint_motor_test::JointMotorTestRunner runner;
        runner.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[joint_motor_test] fatal error: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
