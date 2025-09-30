#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "dynamic_params.hpp"
#include "orbbec_lidar/orbbec_lidar.hpp"

namespace ob_lidar {

class OrbbecLidarNode {
 public:
  OrbbecLidarNode(rclcpp::Node *node, std::shared_ptr<Device> device,
                  std::shared_ptr<Parameters> parameters, bool use_intra_process = false);

  ~OrbbecLidarNode() noexcept;

  void initialize();

  void start();

  void setupDevice();

  void setupTopics();

  void getParameters();

  void clearParameters();

 private:
  void newFrameCallback(const std::shared_ptr<Frame> &frame);

  void publishScan(const std::shared_ptr<Frame> &frame);

  void publishPointCloud(const std::shared_ptr<Frame> &frame);

  // filter scan by parameters max_angle_, min_angle_, max_range_, min_range_
  void filterScan(sensor_msgs::msg::LaserScan &scan);

  // filter point cloud by parameters max_angle_, min_angle_, max_range_, min_range_
  sensor_msgs::msg::PointCloud2 filterPointCloud(sensor_msgs::msg::PointCloud2 &point_cloud) const;

  rclcpp::Node *node_ = nullptr;
  rclcpp::Logger logger_;
  std::shared_ptr<Device> device_;
  DeviceInfo device_info_;
  std::shared_ptr<StreamConfig> stream_config_;
  std::shared_ptr<Parameters> parameters_;
  bool use_intra_process_ = false;
  std::string lidar_name_;
  std::string frame_id_;
  std::string data_type_;
  std::string data_qos_;
  std::string config_file_;
  std::string type_;
  double min_angle_;
  double max_angle_;
  double min_range_;
  double max_range_;
  double scan_frequency_;
  int filter_level_;
  bool use_hardware_time_ = false;
  bool enable_smoothing_filter_ = false;
  std::vector<std::string> parameters_names_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  std::string scan_topic_;
  std::string point_cloud_topic_;
};

}  // namespace ob_lidar
