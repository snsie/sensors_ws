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
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <list>
#include <map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>
#include "ros_param_backend.hpp"
#include "utils.hpp"

namespace ob_lidar {
class Parameters {
 public:
  explicit Parameters(rclcpp::Node *node);

  ~Parameters();
  template <class T>
  T setParam(const std::string &param_name, const T &initial_value,
             std::function<void(const rclcpp::Parameter &)> func =
                 std::function<void(const rclcpp::Parameter &)>(),
             rcl_interfaces::msg::ParameterDescriptor descriptor =
                 rcl_interfaces::msg::ParameterDescriptor());

  template <class T>
  T readAndDeleteParam(const std::string &param_name, const T &initial_value);

  template <class T>
  void setParamT(const std::string &param_name, T &param,
                 std::function<void(const rclcpp::Parameter &)> func =
                     std::function<void(const rclcpp::Parameter &)>(),
                 rcl_interfaces::msg::ParameterDescriptor descriptor =
                     rcl_interfaces::msg::ParameterDescriptor());
  /**
   * function updates the parameter value both
   * locally and in the parameters server
   */
  template <class T>
  void setParamValue(T &param, const T &value);

  // function updates the parameters server
  void setRosParamValue(const std::string &param_name, void const *const value);

  void removeParam(const std::string &param_name);

  void pushUpdateFunctions(std::vector<std::function<void()>> funcs);

  template <class T>
  void queueSetRosValue(const std::string &param_name, const T value);

  template <class T>
  T getParam(const std::string &param_name);

 private:
  void monitorUpdateFunctions();

  rclcpp::Node *node_ = nullptr;
  rclcpp::Logger logger_;
  std::map<std::string, std::function<void(const rclcpp::Parameter &)>> param_functions_;
  std::map<void *, std::string> param_names_;
  ParametersBackend params_backend_;
  std::condition_variable update_functions_cv_;
  std::atomic<bool> is_running_{false};
  std::shared_ptr<std::thread> update_functions_thread_ = nullptr;
  std::deque<std::function<void()>> update_functions_queue_;
  std::list<std::string> self_set_parameters_;
  std::mutex mutex_;
};
}  // namespace ob_lidar
