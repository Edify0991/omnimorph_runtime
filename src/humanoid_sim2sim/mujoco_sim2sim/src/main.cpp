#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "mujoco_sim2sim/mujoco_sim_bridge.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = mujoco_sim2sim::createMujocoSimBridgeNode();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
