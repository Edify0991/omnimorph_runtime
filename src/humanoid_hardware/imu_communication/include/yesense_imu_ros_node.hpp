#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "yesense_imu_decode/analysis_data.h"
#include "yesense_imu_decode/yesense_main.h"

using namespace std::chrono_literals;

class YesenseImuNode : public rclcpp::Node {
 public:
  YesenseImuNode()
      : Node("YesenseImuNode", rclcpp::NodeOptions().use_intra_process_comms(true)) {
    const std::string pub_name_yesense_imu = "/imu/yesense";
    imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_name_yesense_imu, 10);
    imu_run_timer_ = this->create_wall_timer(1ms, std::bind(&YesenseImuNode::timer_callback, this));
    yesense_imu_ = std::make_shared<YesenseImu>();
  }

 private:
  protocol_info_t g_output_info;
  void timer_callback();
  void print_imu_data(const sensor_msgs::msg::Imu::UniquePtr& data);

  rclcpp::TimerBase::SharedPtr imu_run_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  std::shared_ptr<YesenseImu> yesense_imu_;
  int printCount = 0;
};
