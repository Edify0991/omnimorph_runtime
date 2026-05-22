#ifndef MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
#define MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace mujoco_sim2sim
{

class MujocoSimBridge;

// Public factory for the MuJoCo sim2sim ROS node. The concrete bridge class is
// implemented in package-private source files to keep internal state out of the
// installed API surface.
std::shared_ptr<rclcpp::Node> createMujocoSimBridgeNode();

} // namespace mujoco_sim2sim

#endif // MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
