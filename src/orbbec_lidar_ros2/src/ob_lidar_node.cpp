#include <utility>

#include "orbbec_lidar_ros2/constant.hpp"
#include "orbbec_lidar_ros2/ob_lidar_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace ob_lidar {

OrbbecLidarNode::OrbbecLidarNode(rclcpp::Node *node, std::shared_ptr<ob_lidar::Device> device,
                                 std::shared_ptr<Parameters> parameters, bool use_intra_process)
    : node_(node),
      logger_(node_->get_logger()),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      use_intra_process_(use_intra_process) {
  initialize();
}

OrbbecLidarNode::~OrbbecLidarNode() noexcept { clearParameters(); }

void OrbbecLidarNode::initialize() {
  RCLCPP_INFO(logger_, "initialize...");
  getParameters();
  setupDevice();
  setupTopics();
  start();
  RCLCPP_INFO(logger_, "initialize done.");
}

void OrbbecLidarNode::start() {
  RCLCPP_INFO(logger_, "start device with %f Hz", scan_frequency_);
  device_->start(stream_config_,
                 [this](const std::shared_ptr<Frame> &frame) { newFrameCallback(frame); });
  RCLCPP_INFO(logger_, "start device done.");
}

void OrbbecLidarNode::setupDevice() {
  stream_config_ = std::make_shared<StreamConfig>();
  try {
    ob_lidar::setScanSpeed(device_, scan_frequency_);
    stream_config_->enableStream(LidarStreamType::SCAN, scan_frequency_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to enable scan stream: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed to enable scan stream");
  }

  try {
    RCLCPP_INFO(logger_, "setting filter level to %d", filter_level_);
    ob_lidar::setFilterLevel(device_, filter_level_);
    int filter_level = device_->getIntOption(OB_LIDAR_OPTION_FILTER_LEVEL);
    if (filter_level == filter_level_) {
      RCLCPP_INFO(logger_, "set filter level done, filter_level: %d", filter_level);
    } else {
      RCLCPP_WARN(logger_, "Failed to set filter level to %d", filter_level_);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(logger_, "Failed to set filter level: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger_, "Failed to set filter level");
  }
  // TODO: set up enable point cloud when multi-line lidar is supported
}

void OrbbecLidarNode::setupTopics() {
  rmw_qos_profile_t qos = getRMWQosProfileFromString(data_qos_);
  if (use_intra_process_) {
    // NOTE: If we set use_intra_process to true, qos only can be rmw_qos_profile_default
    qos = rmw_qos_profile_default;
  }
  auto cpp_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos));
  if (data_type_ == "laserscan") {
    scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, cpp_qos);
  } else if (data_type_ == "pointcloud") {
    point_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, cpp_qos);
  } else {
    RCLCPP_ERROR(logger_, "Invalid data type: %s", data_type_.c_str());
  }
}

void OrbbecLidarNode::getParameters() {
  RCLCPP_INFO(logger_, "get parameters...");
  std::string param_name = "lidar_name";
  // TODO: rename setParam to good name
  lidar_name_ = parameters_->setParam<std::string>(param_name, "lidar");
  parameters_names_.push_back(param_name);
  param_name = "type";
  type_ = parameters_->setParam<std::string>(param_name, "single_line");
  parameters_names_.push_back(param_name);
  param_name = "frame_id";
  frame_id_ = parameters_->setParam<std::string>(param_name, "scan");
  parameters_names_.push_back(param_name);
  param_name = "data_type";
  data_type_ = parameters_->setParam<std::string>(param_name, "laserscan");
  parameters_names_.push_back(param_name);
  param_name = "data_qos";
  data_qos_ = parameters_->setParam<std::string>(param_name, "sensor_data");
  parameters_names_.push_back(param_name);
  param_name = "min_angle";
  min_angle_ = parameters_->setParam<double>(param_name, -135.0);
  parameters_names_.push_back(param_name);
  param_name = "max_angle";
  max_angle_ = parameters_->setParam<double>(param_name, 135.0);
  parameters_names_.push_back(param_name);
  param_name = "min_range";
  min_range_ = parameters_->setParam<double>(param_name, 0.15);
  parameters_names_.push_back(param_name);
  param_name = "max_range";
  max_range_ = parameters_->setParam<double>(param_name, 30.0);
  parameters_names_.push_back(param_name);
  param_name = "scan_frequency";
  scan_frequency_ = parameters_->setParam<double>(param_name, 30.0);
  parameters_names_.push_back(param_name);
  param_name = "scan_topic";
  scan_topic_ = parameters_->setParam<std::string>(param_name, "scan");
  param_name = "point_cloud_topic";
  point_cloud_topic_ = parameters_->setParam<std::string>(param_name, "point_cloud");
  param_name = "use_hardware_time";
  use_hardware_time_ = parameters_->setParam<bool>(param_name, false);
  param_name = "filter_level";
  filter_level_ = parameters_->setParam<int>(param_name, 0);
  param_name = "enable_smoothing_filter";
  enable_smoothing_filter_ = parameters_->setParam<bool>(param_name, false);
  RCLCPP_INFO(logger_, "get parameters done.");
}

void OrbbecLidarNode::clearParameters() {
  while (!parameters_names_.empty()) {
    parameters_->removeParam(parameters_names_.back());
    parameters_names_.pop_back();
  }
}

void OrbbecLidarNode::newFrameCallback(const std::shared_ptr<Frame> &frame) {
  if (frame == nullptr) {
    return;
  }
  RCLCPP_INFO_ONCE(logger_, "New frame received");
  if (frame->type() == LidarFrameType::SCAN && data_type_ == "laserscan") {
    publishScan(frame);
  } else {
    publishPointCloud(frame);
  }
}

void OrbbecLidarNode::publishScan(const std::shared_ptr<Frame> &frame) {
  if (frame == nullptr) {
    return;
  }
  if (frame->type() != LidarFrameType::SCAN) {
    return;
  }
  auto scan_frame = std::static_pointer_cast<ScanFrame>(frame);
  if (scan_frame == nullptr) {
    return;
  }
  auto scan = scan_frame->toScan();
  if (enable_smoothing_filter_) {
    RCLCPP_INFO_ONCE(logger_, "Starting scan smoothing.");
    uint8_t window_size = 5;
    scan_frame->filterScanSmoothly(scan.ranges, window_size);
  }
  auto timestamp = use_hardware_time_ ? rclcpp::Time(scan.timestamp) : node_->now();
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  auto start_angle = deg2rad(scan.start_angle);
  auto end_angle = deg2rad(scan.end_angle);
  auto angle_increment = scan.angle_resolution;
  scan_msg->header.stamp = timestamp;
  scan_msg->header.frame_id = frame_id_;
  scan_msg->angle_min = static_cast<float>(start_angle);
  scan_msg->angle_max = static_cast<float>(end_angle);
  scan_msg->angle_increment = deg2rad(angle_increment);
  scan_msg->time_increment = 1.0 / scan_frequency_ / scan.ranges.size();
  scan_msg->scan_time = 1.0 / scan_frequency_;
  scan_msg->range_min = min_range_;
  scan_msg->range_max = max_range_;
  scan_msg->ranges.resize(scan.ranges.size());
  scan_msg->intensities.resize(scan.intensities.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    scan_msg->ranges[i] = scan.ranges[i] / 1000.0;
    scan_msg->intensities[i] = scan.intensities[i];
  }
  filterScan(*scan_msg);
  scan_pub_->publish(std::move(scan_msg));
}

void OrbbecLidarNode::publishPointCloud(const std::shared_ptr<Frame> &frame) {
  if (frame == nullptr) {
    return;
  }
  LidarPointCloud point_cloud;
  if (frame->type() == LidarFrameType::SCAN) {
    auto scan_frame = std::static_pointer_cast<ScanFrame>(frame);
    if (scan_frame == nullptr) {
      return;
    }
    auto scan = scan_frame->toScan();
    if (enable_smoothing_filter_) {
      RCLCPP_INFO_ONCE(logger_, "Starting scan smoothing.");
      uint8_t window_size = 5;
      scan_frame->filterScanSmoothly(scan.ranges, window_size);
    }
    point_cloud = scanToPointCloud(scan);
  } else if (frame->type() == LidarFrameType::POINT_CLOUD) {
    auto point_cloud_frame = std::static_pointer_cast<PointCloudFrame>(frame);
    if (point_cloud_frame == nullptr) {
      return;
    }
    point_cloud = point_cloud_frame->toPointCloud();
  }
  auto point_cloud_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  // x, y, z, intensity
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "z", 1,
                                sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
                                sensor_msgs::msg::PointField::UINT8);
  modifier.resize(point_cloud.points.size());
  auto timestamp = use_hardware_time_ ? rclcpp::Time(point_cloud.timestamp) : node_->now();
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_;
  point_cloud_msg->height = 1;
  point_cloud_msg->width = point_cloud.points.size();
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  for (size_t i = 0; i < point_cloud.points.size();
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    *iter_x = static_cast<float>(point_cloud.points[i].x / 1000.0);
    *iter_y = static_cast<float>(point_cloud.points[i].y / 1000.0);
    *iter_z = static_cast<float>(point_cloud.points[i].z / 1000.0);
    *iter_intensity = point_cloud.points[i].reflectivity;
  }
  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(std::move(point_cloud_msg));
}

void OrbbecLidarNode::filterScan(sensor_msgs::msg::LaserScan &scan) {
  double current_angle = scan.angle_min;
  double max_angle = deg2rad(max_angle_);
  double min_angle = deg2rad(min_angle_);
  // map to 0 - 2 * M_PI
  max_angle = std::fmod(max_angle + M_PI, 2 * M_PI);
  if (max_angle < 0) {
    max_angle += 2 * M_PI;
  }

  min_angle = std::fmod(min_angle + M_PI, 2 * M_PI);
  if (min_angle < 0) {
    min_angle += 2 * M_PI;
  }
  if (min_angle > max_angle) {
    std::swap(min_angle, max_angle);
  }
  for (size_t i = 0; i < scan.ranges.size(); ++i, current_angle += scan.angle_increment) {
    bool is_angle_in_range = (current_angle >= min_angle && current_angle <= max_angle);

    bool is_range_in_range = (scan.ranges[i] >= min_range_ && scan.ranges[i] <= max_range_);

    if (!(is_angle_in_range && is_range_in_range)) {
      scan.ranges[i] = 0;
      scan.intensities[i] = 0;
    }
  }
}

sensor_msgs::msg::PointCloud2 OrbbecLidarNode::filterPointCloud(
    sensor_msgs::msg::PointCloud2 &point_cloud) const {
  // Initialize the filtered point cloud
  sensor_msgs::msg::PointCloud2 filtered_point_cloud;
  filtered_point_cloud.header = point_cloud.header;
  filtered_point_cloud.height = point_cloud.height;
  filtered_point_cloud.width = point_cloud.width;
  filtered_point_cloud.is_dense = point_cloud.is_dense;
  filtered_point_cloud.is_bigendian = point_cloud.is_bigendian;
  filtered_point_cloud.fields = point_cloud.fields;
  filtered_point_cloud.point_step = point_cloud.point_step;

  // Convert filter angles from degrees to radians and normalize to [0, 2π]
  double max_angle = deg2rad(max_angle_);
  double min_angle = deg2rad(min_angle_);
  max_angle = std::fmod(max_angle + M_PI, 2 * M_PI);
  min_angle = std::fmod(min_angle + M_PI, 2 * M_PI);
  if (min_angle < 0) {
    min_angle += 2 * M_PI;
  }
  if (max_angle < 0) {
    max_angle += 2 * M_PI;
  }

  // Swap angles if min is greater than max
  if (min_angle > max_angle) {
    std::swap(min_angle, max_angle);
  }

  // Reserve space for filtered point cloud data
  filtered_point_cloud.data.reserve(point_cloud.data.size());

  // Create iterators for each field
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(point_cloud, "intensity");

  // Process each point
  for (size_t i = 0; i < point_cloud.height * point_cloud.width;
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Calculate distance from origin
    float distance = std::sqrt(x * x + y * y + z * z);

    // Calculate angle and normalize to [0, 2π]
    float angle = std::atan2(y, x);
    angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);

    // Check if point is within both angle and range limits
    bool is_angle_in_range = (angle >= min_angle && angle <= max_angle);
    bool is_range_in_range = (distance >= min_range_ && distance <= max_range_);

    if (is_angle_in_range && is_range_in_range) {
      // Keep points within the specified range
      filtered_point_cloud.data.insert(filtered_point_cloud.data.end(),
                                       point_cloud.data.begin() + i * point_cloud.point_step,
                                       point_cloud.data.begin() + (i + 1) * point_cloud.point_step);
    } else {
      // Fill zero values for filtered out points
      filtered_point_cloud.data.insert(filtered_point_cloud.data.end(), point_cloud.point_step, 0);
    }
  }

  // Update row step and resize data
  filtered_point_cloud.row_step = filtered_point_cloud.width * filtered_point_cloud.point_step;
  filtered_point_cloud.data.resize(filtered_point_cloud.height * filtered_point_cloud.row_step);

  return filtered_point_cloud;
}
}  // namespace ob_lidar
