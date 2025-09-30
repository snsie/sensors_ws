// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ob_lidar {
class ParametersBackend {
 public:
  explicit ParametersBackend(rclcpp::Node *node);

  ~ParametersBackend();

#if defined(RCLCPP_HAS_OnSetParametersCallbackType)
  using ros2_param_callback_type =
      rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType;
#else
  using ros2_param_callback_type =
      rclcpp::node_interfaces::NodeParametersInterface::OnParametersSetCallbackType;
#endif

  void addOnSetParametersCallback(ros2_param_callback_type callback);

 private:
  rclcpp::Node *node_;
  rclcpp::Logger logger_;
  std::shared_ptr<void> callback_;
};
}  // namespace ob_lidar
