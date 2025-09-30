#include "orbbec_lidar_ros2/utils.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ob_lidar {
rmw_qos_profile_t getRMWQosProfileFromString(const std::string &str_qos) {
  std::string upper_str_qos = str_qos;
  std::transform(upper_str_qos.begin(), upper_str_qos.end(), upper_str_qos.begin(), ::toupper);
  if (upper_str_qos == "SYSTEM_DEFAULT") {
    return rmw_qos_profile_system_default;
  } else if (upper_str_qos == "DEFAULT") {
    return rmw_qos_profile_default;
  } else if (upper_str_qos == "PARAMETER_EVENTS") {
    return rmw_qos_profile_parameter_events;
  } else if (upper_str_qos == "SERVICES_DEFAULT") {
    return rmw_qos_profile_services_default;
  } else if (upper_str_qos == "PARAMETERS") {
    return rmw_qos_profile_parameters;
  } else if (upper_str_qos == "SENSOR_DATA") {
    return rmw_qos_profile_sensor_data;
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("lidar_camera"),
                        "Invalid QoS profile: " << upper_str_qos << ". Using default QoS profile.");
    return rmw_qos_profile_default;
  }
}

double deg2rad(double deg) { return deg * M_PI / 180.0; }

double rad2deg(double rad) {
  double angle_degrees = rad * (180.0 / M_PI);
  if (angle_degrees < 0) {
    angle_degrees += 360.0;
  }
  return angle_degrees;
}

LidarPointCloud scanToPointCloud(const LidarScan &scan) {
  LidarPointCloud point_cloud{};
  point_cloud.timestamp = scan.timestamp;
  point_cloud.points.reserve(scan.ranges.size());
  auto start_angle = scan.start_angle;
  auto angle_resolution = scan.angle_resolution;
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    LidarPoint point{};
    auto angle = start_angle + angle_resolution * i;
    double rad = deg2rad(angle);
    point.x = scan.ranges[i] * cos(rad);
    point.y = scan.ranges[i] * sin(rad);
    point.z = 0;
    point.reflectivity = scan.intensities[i];
    point_cloud.points.push_back(point);
  }
  return point_cloud;
}

rclcpp::Time rosTimeFromNs(uint64_t ns) { return rclcpp::Time(ns); }

}  // namespace ob_lidar
