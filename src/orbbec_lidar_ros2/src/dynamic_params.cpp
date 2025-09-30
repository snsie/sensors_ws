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

#include "orbbec_lidar_ros2/dynamic_params.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ob_lidar {
Parameters::Parameters(rclcpp::Node *node)
    : node_(node), logger_(node->get_logger()), params_backend_(node), is_running_(true) {
  params_backend_.addOnSetParametersCallback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &parameter : parameters) {
          try {
            const auto &func_iter = param_functions_.find(parameter.get_name());
            if (func_iter != param_functions_.end()) {
              auto name_iter = std::find(self_set_parameters_.begin(), self_set_parameters_.end(),
                                         parameter.get_name());
              if (name_iter != self_set_parameters_.end()) {
                self_set_parameters_.erase(name_iter);
              } else {
                (func_iter->second)(parameter);
              }
            }
          } catch (const std::exception &e) {
            result.successful = false;
            result.reason = e.what();
            RCLCPP_WARN_STREAM(
                logger_, "Set parameter {" << parameter.get_name() << "} failed: " << e.what());
          }
        }

        return result;
      });
  monitorUpdateFunctions();  // Start parameters update thread
}

// pushUpdateFunctions:
// Cannot update ros parameters from within ros parameters callback.
// This function is used by the parameter callback function to update other ros
// parameters.
void Parameters::pushUpdateFunctions(std::vector<std::function<void()>> funcs) {
  update_functions_queue_.insert(update_functions_queue_.end(), funcs.begin(), funcs.end());
  update_functions_cv_.notify_one();
}

void Parameters::monitorUpdateFunctions() {
  int time_interval(1000);
  std::function<void()> func = [this, time_interval]() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (is_running_) {
      update_functions_cv_.wait_for(lock, std::chrono::milliseconds(time_interval), [&] {
        return !is_running_ || !update_functions_queue_.empty();
      });
      while (!update_functions_queue_.empty()) {
        update_functions_queue_.front()();
        update_functions_queue_.pop_front();
      }
    }
  };
  update_functions_thread_ = std::make_shared<std::thread>(func);
}

Parameters::~Parameters() {
  is_running_ = false;
  if (update_functions_thread_ && update_functions_thread_->joinable())
    update_functions_thread_->join();
  for (auto const &param : param_functions_) {
    node_->undeclare_parameter(param.first);
  }
  // remove_on_set_parameters_callback(_params_backend);
}

template <class T>
T Parameters::readAndDeleteParam(const std::string &param_name, const T &initial_value) {
  // Function is meant for reading parameters needed in initialization but
  // should not be declared by the app.
  T result_value = setParam(param_name, initial_value);
  removeParam(param_name);
  return result_value;
}

template <class T>
T Parameters::setParam(const std::string &param_name, const T &initial_value,
                       std::function<void(const rclcpp::Parameter &)> func,
                       rcl_interfaces::msg::ParameterDescriptor descriptor) {
  T result_value(initial_value);
  try {
    RCLCPP_DEBUG_STREAM(logger_, "setParam::Setting parameter: " << param_name);
#if defined(FOXY)
    // do nothing for old versions (foxy)
#else
    descriptor.dynamic_typing = true;
#endif
    // if the parameter is not set, declare it
    if (!node_->get_parameter(param_name, result_value)) {
      result_value = node_->declare_parameter(param_name, initial_value, descriptor);
    }
  } catch (const std::exception &e) {
    std::stringstream range;
    for (auto val : descriptor.floating_point_range) {
      range << val.from_value << ", " << val.to_value;
    }
    for (auto val : descriptor.integer_range) {
      range << val.from_value << ", " << val.to_value;
    }
    RCLCPP_WARN_STREAM(logger_, "Could not set param: " << param_name << " with " << initial_value
                                                        << " Range: [" << range.str()
                                                        << "] : " << e.what());
    return initial_value;
  }

  if (param_functions_.find(param_name) != param_functions_.end()) {
    RCLCPP_INFO_STREAM(logger_, "setParam::Replace function for : " << param_name);
  }

  if (func) {
    param_functions_[param_name] = func;
  } else {
    param_functions_[param_name] = [this](const rclcpp::Parameter &) {
      RCLCPP_WARN_STREAM(logger_, "Parameter can not be changed in runtime.");
    };
  }
  if (result_value != initial_value && func) {
    try {
      func(rclcpp::Parameter(param_name, result_value));
    } catch (const std::exception &e) {
      RCLCPP_WARN_STREAM(logger_, "Set parameter {" << param_name << "} failed: " << e.what());
    }
  }
  return result_value;
}

// setParamT: Used to automatically update param based on its parallel ros
// parameter. Notice: param must remain alive as long as the callback is active
// -
//      if param is destroyed the behavior of the callback is undefined.
template <class T>
void Parameters::setParamT(const std::string &param_name, T &param,
                           std::function<void(const rclcpp::Parameter &)> func,
                           rcl_interfaces::msg::ParameterDescriptor descriptor)

{
  param = setParam<T>(
      param_name, param,
      [&param, func](const rclcpp::Parameter &parameter) {
        param = parameter.get_value<T>();
        if (func) func(parameter);
      },
      descriptor);
}

template <class T>
void Parameters::setParamValue(T &param, const T &value) {
  // setParamValue updates a variable and its parallel in the parameters server.
  // NOTICE: <param> must have the same address it was declared with.
  param = value;
  try {
    std::string param_name = param_names_.at(&param);

    rcl_interfaces::msg::SetParametersResult results =
        node_->set_parameter(rclcpp::Parameter(param_name, value));
    if (!results.successful) {
      RCLCPP_WARN_STREAM(logger_,
                         "Parameter: " << param_name << " was not set: " << results.reason);
    }
  } catch (const std::out_of_range &e) {
    RCLCPP_WARN_STREAM(logger_, "Parameter was not internally declared.");
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException &e) {
    std::string param_name = param_names_.at(&param);
    RCLCPP_WARN_STREAM(logger_, "Parameter: " << param_name << " was not declared: " << e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(logger_, __FILE__ << ":" << __LINE__ << ":" << e.what());
  }
}

// setRosParamValue - Used to set ROS parameter back to a valid value if an
// invalid value was set by user.
void Parameters::setRosParamValue(const std::string &param_name, void const *const value) {
  // setRosParamValue sets a value to a parameter in the parameters server.
  // The callback for the specified parameter is NOT called.
  self_set_parameters_.push_back(param_name);

  rclcpp::ParameterType param_type = node_->get_parameter(param_name).get_type();
  rcl_interfaces::msg::SetParametersResult results;
  switch (param_type) {
    case rclcpp::PARAMETER_BOOL:
      RCLCPP_INFO_STREAM(logger_, "Set " << param_name << " to " << *(bool *)value);
      results = node_->set_parameter(rclcpp::Parameter(param_name, *(bool *)value));
      break;
    case rclcpp::PARAMETER_INTEGER:
      RCLCPP_INFO_STREAM(logger_, "Set " << param_name << " to " << *(int *)value);
      results = node_->set_parameter(rclcpp::Parameter(param_name, *(int *)value));
      break;
    case rclcpp::PARAMETER_DOUBLE:
      RCLCPP_INFO_STREAM(logger_, "Set " << param_name << " to " << *(double *)value);
      results = node_->set_parameter(rclcpp::Parameter(param_name, *(double *)value));
      break;
    case rclcpp::PARAMETER_STRING:
      RCLCPP_INFO_STREAM(logger_, "Set " << param_name << " to " << *(std::string *)value);
      results = node_->set_parameter(rclcpp::Parameter(param_name, *(std::string *)value));
      break;
    default:
      RCLCPP_ERROR_STREAM(logger_, "Setting parameter of type "
                                       << node_->get_parameter(param_name).get_type_name()
                                       << " is not implemented.");
  }
  if (!results.successful) {
    RCLCPP_WARN_STREAM(logger_, "Parameter: " << param_name << " was not set: " << results.reason);
    self_set_parameters_.pop_back();
  }
}

// queueSetRosValue - Set parameter in queue to be pushed to ROS parameter by
// monitor_update_functions
template <class T>
void Parameters::queueSetRosValue(const std::string &param_name, const T value) {
  std::vector<std::function<void()>> funcs;
  funcs.push_back([this, param_name, value]() { setRosParamValue(param_name, &value); });
  pushUpdateFunctions(funcs);
}

void Parameters::removeParam(const std::string &param_name) {
  if (node_->has_parameter(param_name)) {
    node_->undeclare_parameter(param_name);
  }
  param_functions_.erase(param_name);
}

template <class T>
T Parameters::getParam(const std::string &param_name) {
  return node_->get_parameter(param_name).get_value<T>();
}

template void Parameters::setParamT<bool>(const std::string &param_name, bool &param,
                                          std::function<void(const rclcpp::Parameter &)> func,
                                          rcl_interfaces::msg::ParameterDescriptor descriptor);
template void Parameters::setParamT<int>(const std::string &param_name, int &param,
                                         std::function<void(const rclcpp::Parameter &)> func,
                                         rcl_interfaces::msg::ParameterDescriptor descriptor);
template void Parameters::setParamT<double>(const std::string &param_name, double &param,
                                            std::function<void(const rclcpp::Parameter &)> func,
                                            rcl_interfaces::msg::ParameterDescriptor descriptor);

template bool Parameters::setParam<bool>(const std::string &param_name, const bool &initial_value,
                                         std::function<void(const rclcpp::Parameter &)> func,
                                         rcl_interfaces::msg::ParameterDescriptor descriptor);
template int Parameters::setParam<int>(const std::string &param_name, const int &initial_value,
                                       std::function<void(const rclcpp::Parameter &)> func,
                                       rcl_interfaces::msg::ParameterDescriptor descriptor);

template double Parameters::setParam<double>(const std::string &param_name,
                                             const double &initial_value,
                                             std::function<void(const rclcpp::Parameter &)> func,
                                             rcl_interfaces::msg::ParameterDescriptor descriptor);

template std::string Parameters::setParam<std::string>(
    const std::string &param_name, const std::string &initial_value,
    std::function<void(const rclcpp::Parameter &)> func,
    rcl_interfaces::msg::ParameterDescriptor descriptor);

template void Parameters::setParamValue<int>(int &param, const int &value);

template void Parameters::setParamValue<bool>(bool &param, const bool &value);

template void Parameters::setParamValue<double>(double &param, const double &value);

template void Parameters::queueSetRosValue<std::string>(const std::string &param_name,
                                                        const std::string value);
template void Parameters::queueSetRosValue<int>(const std::string &param_name, const int value);

template int Parameters::readAndDeleteParam<int>(const std::string &param_name,
                                                 const int &initial_value);

template bool Parameters::getParam<bool>(const std::string &param_name);
}  // namespace ob_lidar
