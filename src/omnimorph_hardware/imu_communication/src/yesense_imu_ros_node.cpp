#include "yesense_imu_ros_node.hpp"

#include <cmath>
#include <iostream>

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

void YesenseImuNode::print_imu_data(const sensor_msgs::msg::Imu::UniquePtr& data) {
  std::cout << "--------------------------------Yesense Imu Data--------------------------------" << std::endl;
  std::cout << "IMU roll : " << data->orientation.x << " [rad]" << std::endl;
  std::cout << "IMU pitch: " << data->orientation.y << " [rad]" << std::endl;
  std::cout << "IMU yaw  : " << data->orientation.z << " [rad]" << std::endl;
  std::cout << "IMU acc x: " << data->linear_acceleration.x << " [m/s^2]" << std::endl;
  std::cout << "IMU acc y: " << data->linear_acceleration.y << " [m/s^2]" << std::endl;
  std::cout << "IMU acc z: " << data->linear_acceleration.z << " [m/s^2]" << std::endl;
  std::cout << "IMU wx   : " << data->angular_velocity.x << " [rad/s]" << std::endl;
  std::cout << "IMU wy   : " << data->angular_velocity.y << " [rad/s]" << std::endl;
  std::cout << "IMU wz   : " << data->angular_velocity.z << " [rad/s]" << std::endl;
}

void YesenseImuNode::timer_callback() {
  if (!yesense_imu_->updateImuData()) {
    return;
  }

  sensor_msgs::msg::Imu::UniquePtr imu_data_(new sensor_msgs::msg::Imu());
  imu_data_->header.stamp = this->now();
  imu_data_->header.frame_id = "world";

  g_output_info = yesense_imu_->getImuData();

  imu_data_->angular_velocity.x = g_output_info.angle_rate.x * kDegToRad;
  imu_data_->angular_velocity.y = g_output_info.angle_rate.y * kDegToRad;
  imu_data_->angular_velocity.z = g_output_info.angle_rate.z * kDegToRad;

  imu_data_->linear_acceleration.x = g_output_info.accel.x;
  imu_data_->linear_acceleration.y = g_output_info.accel.y;
  imu_data_->linear_acceleration.z = g_output_info.accel.z;

  // Keep the existing contract: orientation carries roll/pitch/yaw in radians.
  imu_data_->orientation.w = 0.0;
  imu_data_->orientation.x = g_output_info.attitude.roll * kDegToRad;
  imu_data_->orientation.y = g_output_info.attitude.pitch * kDegToRad;
  imu_data_->orientation.z = g_output_info.attitude.yaw * kDegToRad;

  if (printCount % 500 == 0) {
    print_imu_data(imu_data_);
  }

  this->imu_data_publisher_->publish(std::move(imu_data_));
  printCount += 1;
}
