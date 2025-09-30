#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ob_lidar_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "orbbec_lidar/orbbec_lidar.hpp"

namespace ob_lidar {

class OBLidarDriver : public rclcpp::Node {
 public:
  explicit OBLidarDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  OBLidarDriver(const std::string& node_name, const std::string& ns,
                const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());

  void init();

  ~OBLidarDriver() override;

 private:
  rclcpp::Logger logger_;
  std::shared_ptr<Device> device_ = nullptr;
  bool use_intra_process_ = false;
  std::shared_ptr<DeviceConfig> device_config_ = nullptr;
  std::shared_ptr<Parameters> parameters_ = nullptr;
  std::unique_ptr<OrbbecLidarNode> orbbec_lidar_node_ = nullptr;
  ob_lidar::DeviceInfo device_info_;
  std::string log_level_ = "info";
  std::string config_file_;
};

}  // namespace ob_lidar
