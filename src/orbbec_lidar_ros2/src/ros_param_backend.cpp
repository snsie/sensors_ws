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

#include <utility>

#include "orbbec_lidar_ros2/ros_param_backend.hpp"

namespace ob_lidar {

ParametersBackend::ParametersBackend(rclcpp::Node *node)
    : node_(node), logger_(node->get_logger()) {}

void ParametersBackend::addOnSetParametersCallback(ros2_param_callback_type callback) {
  callback_ = node_->add_on_set_parameters_callback(std::move(callback));
}

ParametersBackend::~ParametersBackend() {
  if (callback_) {
    node_->remove_on_set_parameters_callback(
        static_cast<rclcpp::node_interfaces::OnSetParametersCallbackHandle *>(callback_.get()));
    callback_.reset();
  }
}
}  // namespace ob_lidar
