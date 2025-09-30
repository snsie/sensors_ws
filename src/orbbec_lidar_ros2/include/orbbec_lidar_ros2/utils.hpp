#pragma once

#include <rclcpp/rclcpp.hpp>
#include "orbbec_lidar/orbbec_lidar.hpp"
namespace ob_lidar {
rmw_qos_profile_t getRMWQosProfileFromString(const std::string &str_qos);

double deg2rad(double deg);

double rad2deg(double rad);

LidarPointCloud scanToPointCloud(const LidarScan &scan);

// ros time from nanoseconds
rclcpp::Time rosTimeFromNs(uint64_t ns);

}  // namespace ob_lidar
