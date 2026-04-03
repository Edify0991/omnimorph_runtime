#include "yesense_imu_ros_node.hpp"
#include <vector>

// 数据打印封装函数
void YesenseImuNode::print_imu_data(const sensor_msgs::msg::Imu::UniquePtr& data) {
  std::cout << "--------------------------------Yesense Imu Data--------------------------------" << std::endl;
  std::cout << "IMU角度x:" << data->orientation.x << " [rad]" << std::endl;
  std::cout << "IMU角度y:" << data->orientation.y << " [rad]" << std::endl;
  std::cout << "IMU角度z:" << data->orientation.z << " [rad]" << std::endl;
  std::cout << "IMU线加速度x:" << data->linear_acceleration.x << " [m/s²]" << std::endl;
  std::cout << "IMU线加速度y:" << data->linear_acceleration.y << " [m/s²]" << std::endl;
  std::cout << "IMU线加速度z:" << data->linear_acceleration.z << " [m/s²]" << std::endl;
  std::cout << "IMU角速度x:" << data->angular_velocity.x << " [rad/s]" << std::endl;
  std::cout << "IMU角速度y:" << data->angular_velocity.y << " [rad/s]" << std::endl;
  std::cout << "IMU角速度z:" << data->angular_velocity.z << " [rad/s]" << std::endl;
}

void YesenseImuNode::timer_callback() {
  bool update_ = yesense_imu_->updateImuData();
  if (update_) {
    sensor_msgs::msg::Imu::UniquePtr imu_data_(new sensor_msgs::msg::Imu());
    imu_data_->header.stamp = this->now();
    imu_data_->header.frame_id = "world";
    g_output_info = yesense_imu_->getImuData();
    imu_data_->angular_velocity.x = g_output_info.angle_rate.x / 180 * M_PI;
    imu_data_->angular_velocity.y = g_output_info.angle_rate.y / 180 * M_PI;
    imu_data_->angular_velocity.z = g_output_info.angle_rate.z / 180 * M_PI;
    imu_data_->linear_acceleration.x = g_output_info.accel.x;
    imu_data_->linear_acceleration.y = g_output_info.accel.y;
    imu_data_->linear_acceleration.z = g_output_info.accel.z;
    // 这里不是四元数，只是借用这个格式，实际是欧拉角
    imu_data_->orientation.w = 0;
    imu_data_->orientation.x = g_output_info.attitude.roll / 180.0 * M_PI;
    imu_data_->orientation.y = g_output_info.attitude.pitch / 180.0 * M_PI;
    imu_data_->orientation.z = g_output_info.attitude.yaw / 180.0 * M_PI;

    // xxx Hz更新的Imu数据，根据需要修改 - 按照MTI格式组织数据
    std::vector<float> imuValues;

    // 四元数数据 (w, x, y, z) - 注意：Yesense这里用的是欧拉角，需要转换为四元数或保持一致性
    // 由于原代码中w=0，这里保持原有逻辑，但按MTI顺序排列
    imuValues.push_back(g_output_info.attitude.quaternion_data0);  // w (这里Yesense没有真正的四元数w分量)
    imuValues.push_back(g_output_info.attitude.quaternion_data1);  // x (roll)
    imuValues.push_back(g_output_info.attitude.quaternion_data2);  // y (pitch) 
    imuValues.push_back(g_output_info.attitude.quaternion_data3);  // z (yaw)

    // 欧拉角数据 (roll, pitch, yaw)
    imuValues.push_back(g_output_info.attitude.roll / 180.0 * M_PI);    // roll in radians
    imuValues.push_back(g_output_info.attitude.pitch / 180.0 * M_PI);   // pitch in radians
    imuValues.push_back(g_output_info.attitude.yaw / 180.0 * M_PI);     // yaw in radians

    // 角速度数据 (wx, wy, wz)
    imuValues.push_back(g_output_info.angle_rate.x / 180 * M_PI);  // wx
    imuValues.push_back(g_output_info.angle_rate.y / 180 * M_PI);  // wy
    imuValues.push_back(g_output_info.angle_rate.z / 180 * M_PI);  // wz

    // 可选：加速度数据 (ax, ay, az) - MTI代码中没有，但可以添加
    // imuValues.push_back(imu_data_->linear_acceleration.x);  // ax
    // imuValues.push_back(imu_data_->linear_acceleration.y);  // ay
    // imuValues.push_back(imu_data_->linear_acceleration.z);  // az

    if(printCount % 500 == 0){
      print_imu_data(imu_data_);
      // 打印数据格式改为与MTI一致
      // std::cout << "Yesense IMU Data: ";
      // for(size_t i = 0; i < imuValues.size(); ++i){
      //   std::cout << imuValues[i] << ", ";
      // }
      // std::cout << std::endl;
    }

    // 计算元素数量
    size_t elementCount = imuValues.size();

    // Imu数据更新到共享内存中 - 按照MTI方式写入
    imuValuesSm->write(&imuValues[0], elementCount, 0);

    this->imu_data_publisher_->publish(std::move(imu_data_));
    printCount += 1;
  }
}
