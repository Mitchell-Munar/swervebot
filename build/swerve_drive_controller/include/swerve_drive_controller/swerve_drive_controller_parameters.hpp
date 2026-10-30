// auto-generated DO NOT EDIT

#pragma once

#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <mutex>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp/logger.hpp>
#include <set>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>

#include <rsl/static_string.hpp>
#include <rsl/static_vector.hpp>
#include <rsl/parameter_validators.hpp>



namespace swerve_drive_controller {

// Use validators from RSL
using rsl::unique;
using rsl::subset_of;
using rsl::fixed_size;
using rsl::size_gt;
using rsl::size_lt;
using rsl::not_empty;
using rsl::element_bounds;
using rsl::lower_element_bounds;
using rsl::upper_element_bounds;
using rsl::bounds;
using rsl::lt;
using rsl::gt;
using rsl::lt_eq;
using rsl::gt_eq;
using rsl::one_of;
using rsl::to_parameter_result_msg;

template <typename T>
[[nodiscard]] auto to_parameter_value(T value) {
    return rclcpp::ParameterValue(value);
}

template <size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticString<capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_string(value));
}

template <typename T, size_t capacity>
[[nodiscard]] auto to_parameter_value(rsl::StaticVector<T, capacity> const& value) {
    return rclcpp::ParameterValue(rsl::to_vector(value));
}
    struct Params {
        std::vector<std::string> wheel_names;
        std::vector<std::string> actuator_names;
        std::vector<std::string> wheel_command_interfaces = {"velocity"};
        std::vector<std::string> wheel_state_interfaces = {"velocity"};
        std::vector<std::string> actuator_command_interfaces = {"position"};
        std::vector<std::string> actuator_state_interfaces = {"position", "velocity"};
        bool enable_odom_tf = true;
        std::string odom_frame_id = "odom";
        std::string base_frame_id = "base_footprint";
        double cmd_vel_timeout = 0.5;
        std::string unsupported_command_policy = "stop";
        std::vector<double> module_x;
        std::vector<double> module_y;
        std::vector<double> wheel_radii;
        std::vector<double> wheel_offsets;
        std::vector<bool> steering_continuous;
        std::vector<double> steering_min;
        std::vector<double> steering_max;
        std::vector<double> steering_velocity_limits;
        double odom_publish_rate = 50.0;
        double command_epsilon = 1e-06;
        double steering_feedback_tolerance = 0.001;
        // for detecting if the parameter struct has been updated
        rclcpp::Time __stamp;
    };
    struct StackParams {
        bool enable_odom_tf = true;
        double cmd_vel_timeout = 0.5;
        double odom_publish_rate = 50.0;
        double command_epsilon = 1e-06;
        double steering_feedback_tolerance = 0.001;
    };

  class ParamListener{
  public:
    // throws rclcpp::exceptions::InvalidParameterValueException on initialization if invalid parameter are loaded
    template <typename NodeT>
    ParamListener(NodeT node, std::string const& prefix = "")
    : ParamListener(node->get_node_parameters_interface(), node->get_logger(), prefix) {}

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  std::string const& prefix = "")
    : ParamListener(parameters_interface, rclcpp::get_logger("swerve_drive_controller"), prefix) {
      RCLCPP_DEBUG(logger_, "ParameterListener: Not using node logger, recommend using other constructors to use a node logger");
    }

    ParamListener(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface,
                  rclcpp::Logger logger, std::string const& prefix = "")
    : prefix_{prefix},
      logger_{std::move(logger)} {
      if (!prefix_.empty() && prefix_.back() != '.') {
        prefix_ += ".";
      }

      parameters_interface_ = parameters_interface;
      declare_params();
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters){return this->update(parameters);};
      handle_ = parameters_interface_->add_on_set_parameters_callback(update_param_cb);
      clock_ = rclcpp::Clock();
    }

    Params get_params() const{
      std::lock_guard<std::mutex> lock(mutex_);
      return params_;
    }

    /**
     * @brief Tries to update the parsed Params object
     * @param params_in The Params object to update
     * @return true if the Params object was updated, false if it was already up to date or the mutex could not be locked
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_update_params(Params & params_in) const {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (lock.owns_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
          return true;
        }
      }
      return false;
    }

    /**
     * @brief Tries to get the current Params object
     * @param params_in The Params object to fill with the current parameters
     * @return true if mutex can be locked, false if mutex could not be locked
     * @note The parameters are only filled, when the mutex can be locked and the params timestamp is different
     * @note This function tries to lock the mutex without blocking, so it can be used in a RT loop
     */
    bool try_get_params(Params & params_in) const {
      if (mutex_.try_lock()) {
        if (const bool is_old = params_in.__stamp != params_.__stamp; is_old) {
          params_in = params_;
        }
        mutex_.unlock();
        return true;
      }
      return false;
    }

    bool is_old(Params const& other) const {
      std::lock_guard<std::mutex> lock(mutex_);
      return params_.__stamp != other.__stamp;
    }

    StackParams get_stack_params() {
      Params params = get_params();
      StackParams output;
      output.enable_odom_tf = params.enable_odom_tf;
      output.cmd_vel_timeout = params.cmd_vel_timeout;
      output.odom_publish_rate = params.odom_publish_rate;
      output.command_epsilon = params.command_epsilon;
      output.steering_feedback_tolerance = params.steering_feedback_tolerance;

      return output;
    }

    void refresh_dynamic_parameters() {
      auto updated_params = get_params();
      // TODO remove any destroyed dynamic parameters

      // declare any new dynamic parameters
      rclcpp::Parameter param;

    }

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      auto updated_params = get_params();

      for (const auto &param: parameters) {
        if (param.get_name() == (prefix_ + "wheel_names")) {
            updated_params.wheel_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "actuator_names")) {
            updated_params.actuator_names = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_command_interfaces")) {
            updated_params.wheel_command_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_state_interfaces")) {
            updated_params.wheel_state_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "actuator_command_interfaces")) {
            updated_params.actuator_command_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "actuator_state_interfaces")) {
            updated_params.actuator_state_interfaces = param.as_string_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "enable_odom_tf")) {
            updated_params.enable_odom_tf = param.as_bool();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_frame_id")) {
            updated_params.odom_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "base_frame_id")) {
            updated_params.base_frame_id = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "cmd_vel_timeout")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = lt_eq<double>(param, 3600.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.cmd_vel_timeout = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "unsupported_command_policy")) {
            if(auto validation_result = one_of<std::string>(param, {"stop", "ignore"});
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.unsupported_command_policy = param.as_string();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_x")) {
            updated_params.module_x = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "module_y")) {
            updated_params.module_y = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_radii")) {
            updated_params.wheel_radii = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "wheel_offsets")) {
            updated_params.wheel_offsets = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_continuous")) {
            updated_params.steering_continuous = param.as_bool_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_min")) {
            updated_params.steering_min = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_max")) {
            updated_params.steering_max = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_velocity_limits")) {
            updated_params.steering_velocity_limits = param.as_double_array();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "odom_publish_rate")) {
            if(auto validation_result = gt<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            if(auto validation_result = lt_eq<double>(param, 1000.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.odom_publish_rate = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "command_epsilon")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.command_epsilon = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
        if (param.get_name() == (prefix_ + "steering_feedback_tolerance")) {
            if(auto validation_result = gt_eq<double>(param, 0.0);
              !validation_result) {
                return rsl::to_parameter_result_msg(validation_result);
            }
            updated_params.steering_feedback_tolerance = param.as_double();
            RCLCPP_DEBUG_STREAM(logger_, param.get_name() << ": " << param.get_type_name() << " = " << param.value_to_string());
        }
      }

      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
      if (user_callback_) {
         user_callback_(updated_params);
      }
      return rsl::to_parameter_result_msg({});
    }

    void declare_params(){
      auto updated_params = get_params();
      // declare all parameters and give default values to non-required ones
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Drive joint names; at least three, in the same order as all module arrays";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "wheel_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "actuator_names")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Steering joint names in module-array order";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_STRING_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "actuator_names", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_command_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Command interface for the wheel joints";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.wheel_command_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "wheel_command_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_state_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "State interface for the wheel joints";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.wheel_state_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "wheel_state_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "actuator_command_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Command interface for the actuator joints";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.actuator_command_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "actuator_command_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "actuator_state_interfaces")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "State interface for the actuator joints";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.actuator_state_interfaces);
          parameters_interface_->declare_parameter(prefix_ + "actuator_state_interfaces", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "enable_odom_tf")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Publish odom to base_footprint transform";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.enable_odom_tf);
          parameters_interface_->declare_parameter(prefix_ + "enable_odom_tf", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Odometry frame id";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.odom_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "odom_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "base_frame_id")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Base robot frame id";
          descriptor.read_only = true;
          auto parameter = to_parameter_value(updated_params.base_frame_id);
          parameters_interface_->declare_parameter(prefix_ + "base_frame_id", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "cmd_vel_timeout")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Maximum command age in monotonic seconds before stopping (up to 3600)";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = std::nextafter(0.0, std::numeric_limits<double>::infinity());
          descriptor.floating_point_range.at(0).to_value = 3600.0;
          auto parameter = to_parameter_value(updated_params.cmd_vel_timeout);
          parameters_interface_->declare_parameter(prefix_ + "cmd_vel_timeout", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "unsupported_command_policy")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Stop rejects nonplanar components; ignore discards them. Changes require a fresh Twist.";
          descriptor.read_only = false;
          auto parameter = to_parameter_value(updated_params.unsupported_command_policy);
          parameters_interface_->declare_parameter(prefix_ + "unsupported_command_policy", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_x")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Steering pivot x coordinates in the base frame (m)";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "module_x", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "module_y")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Steering pivot y coordinates in the base frame (m)";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "module_y", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_radii")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Positive rolling radii in module order (m)";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "wheel_radii", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "wheel_offsets")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Signed offset along local left normal to rolling direction (m)";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "wheel_offsets", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_continuous")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "True for unwrapped continuous steering; false for bounded travel";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_BOOL_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steering_continuous", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_min")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Lower steering positions (rad); ignored for continuous joints";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steering_min", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_max")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Upper steering positions (rad); ignored for continuous joints";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steering_max", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_velocity_limits")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Positive per-module steering slew limits (rad/s)";
          descriptor.read_only = true;
          auto parameter = rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;
          parameters_interface_->declare_parameter(prefix_ + "steering_velocity_limits", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "odom_publish_rate")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Executor odometry publication rate (Hz)";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = std::nextafter(0.0, std::numeric_limits<double>::infinity());
          descriptor.floating_point_range.at(0).to_value = 1000.0;
          auto parameter = to_parameter_value(updated_params.odom_publish_rate);
          parameters_interface_->declare_parameter(prefix_ + "odom_publish_rate", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "command_epsilon")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Components smaller than this threshold are zero (SI units)";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.command_epsilon);
          parameters_interface_->declare_parameter(prefix_ + "command_epsilon", parameter, descriptor);
      }
      if (!parameters_interface_->has_parameter(prefix_ + "steering_feedback_tolerance")) {
          rcl_interfaces::msg::ParameterDescriptor descriptor;
          descriptor.description = "Allowed measured overshoot beyond bounded travel (rad)";
          descriptor.read_only = true;
          descriptor.floating_point_range.resize(1);
          descriptor.floating_point_range.at(0).from_value = 0.0;
          descriptor.floating_point_range.at(0).to_value = std::numeric_limits<double>::max();
          auto parameter = to_parameter_value(updated_params.steering_feedback_tolerance);
          parameters_interface_->declare_parameter(prefix_ + "steering_feedback_tolerance", parameter, descriptor);
      }
      // get parameters and fill struct fields
      rclcpp::Parameter param;
      param = parameters_interface_->get_parameter(prefix_ + "wheel_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "actuator_names");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "actuator_names") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.actuator_names = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_command_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_command_interfaces") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_command_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_state_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_state_interfaces") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_state_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "actuator_command_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "actuator_command_interfaces") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.actuator_command_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "actuator_state_interfaces");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "actuator_state_interfaces") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.actuator_state_interfaces = param.as_string_array();
      param = parameters_interface_->get_parameter(prefix_ + "enable_odom_tf");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "enable_odom_tf") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.enable_odom_tf = param.as_bool();
      param = parameters_interface_->get_parameter(prefix_ + "odom_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "odom_frame_id") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.odom_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "base_frame_id");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "base_frame_id") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.base_frame_id = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "cmd_vel_timeout");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "cmd_vel_timeout") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'cmd_vel_timeout': {}", validation_result.error()));
      }
      if(auto validation_result = lt_eq<double>(param, 3600.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'cmd_vel_timeout': {}", validation_result.error()));
      }
      updated_params.cmd_vel_timeout = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "unsupported_command_policy");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "unsupported_command_policy") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = one_of<std::string>(param, {"stop", "ignore"});
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'unsupported_command_policy': {}", validation_result.error()));
      }
      updated_params.unsupported_command_policy = param.as_string();
      param = parameters_interface_->get_parameter(prefix_ + "module_x");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "module_x") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_x = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "module_y");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "module_y") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.module_y = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_radii");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_radii") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_radii = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "wheel_offsets");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "wheel_offsets") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.wheel_offsets = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_continuous");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steering_continuous") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_continuous = param.as_bool_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_min");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steering_min") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_min = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_max");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steering_max") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_max = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "steering_velocity_limits");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steering_velocity_limits") << ": " << param.get_type_name() << " = " << param.value_to_string());
      updated_params.steering_velocity_limits = param.as_double_array();
      param = parameters_interface_->get_parameter(prefix_ + "odom_publish_rate");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "odom_publish_rate") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'odom_publish_rate': {}", validation_result.error()));
      }
      if(auto validation_result = lt_eq<double>(param, 1000.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'odom_publish_rate': {}", validation_result.error()));
      }
      updated_params.odom_publish_rate = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "command_epsilon");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "command_epsilon") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'command_epsilon': {}", validation_result.error()));
      }
      updated_params.command_epsilon = param.as_double();
      param = parameters_interface_->get_parameter(prefix_ + "steering_feedback_tolerance");
      RCLCPP_DEBUG_STREAM(logger_, (prefix_ + "steering_feedback_tolerance") << ": " << param.get_type_name() << " = " << param.value_to_string());
      if(auto validation_result = gt_eq<double>(param, 0.0);
        !validation_result) {
          throw rclcpp::exceptions::InvalidParameterValueException(fmt::format("Invalid value set during initialization for parameter 'steering_feedback_tolerance': {}", validation_result.error()));
      }
      updated_params.steering_feedback_tolerance = param.as_double();


      updated_params.__stamp = clock_.now();
      update_internal_params(updated_params);
    }

    using userParameterUpdateCB = std::function<void(const Params&)>;
    void setUserCallback(const userParameterUpdateCB& callback){
      user_callback_ = callback;
    }

    void clearUserCallback(){
      user_callback_ = {};
    }

    private:
      void update_internal_params(Params updated_params) {
        std::lock_guard<std::mutex> lock(mutex_);
        params_ = std::move(updated_params);
      }

      std::string prefix_;
      Params params_;
      rclcpp::Clock clock_;
      std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;
      std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface_;
      userParameterUpdateCB user_callback_;

      rclcpp::Logger logger_;
      std::mutex mutable mutex_;
  };

} // namespace swerve_drive_controller
