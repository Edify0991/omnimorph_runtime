#include <iostream>
#include "yesense_imu_ros_node.hpp"
#include "rclcpp/rclcpp.hpp"

#define RESETSTR "\033[0m"
#define REDSTR "\033[31m"   /* Red */
#define GREENSTR "\033[32m" /* Green */

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  bool yesense_enable = true;

  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::Node::SharedPtr yesense_imu_node;

  if (yesense_enable) {
    yesense_imu_node = std::make_shared<YesenseImuNode>();
    executor.add_node(yesense_imu_node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
