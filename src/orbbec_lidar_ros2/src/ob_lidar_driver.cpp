#include <utility>

#include "orbbec_lidar_ros2/ob_lidar_driver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace ob_lidar {
OBLidarDriver::OBLidarDriver(const rclcpp::NodeOptions& options)
    : Node("orbbec_camera_driver", "/", options),
      logger_(this->get_logger()),
      use_intra_process_(options.use_intra_process_comms()) {
  init();
}
OBLidarDriver::OBLidarDriver(const std::string& node_name, const std::string& ns,
                             const rclcpp::NodeOptions& node_options)
    : Node(node_name, ns, node_options),
      logger_(this->get_logger()),
      use_intra_process_(node_options.use_intra_process_comms()) {
  init();
}

void OBLidarDriver::init() {
  log_level_ = this->declare_parameter("log_level", "info");
  config_file_ = this->declare_parameter("config_file", "");
  parameters_ = std::make_shared<Parameters>(this);
  if (config_file_.empty()) {
    RCLCPP_ERROR(logger_, "Config file is not set");
    exit(-1);
  }
  device_ = DeviceFactory::create(config_file_);
  if (device_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to create device");
    exit(-1);
  }
  device_info_ = device_->getInfo();
  orbbec_lidar_node_ =
      std::make_unique<OrbbecLidarNode>(this, device_, parameters_, use_intra_process_);
}

OBLidarDriver::~OBLidarDriver() = default;

}  // namespace ob_lidar

RCLCPP_COMPONENTS_REGISTER_NODE(ob_lidar::OBLidarDriver)
