#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "swervebot_controllers/swerve_controller.hpp"
#include "swervebot_controllers/swerve_kinematics.hpp"

namespace swervebot_controllers
{
namespace
{
// Lyrical command limiters may throw runtime_error; report failure without logging in RT.
bool write_command(hardware_interface::LoanedCommandInterface & interface, double value)
{
  try {
    return interface.set_value(value, 1);
  } catch (const std::runtime_error &) {
    return false;
  }
}
}  // namespace
int64_t SwerveController::steady_now()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

controller_interface::CallbackReturn SwerveController::on_init()
{
  try {
    param_listener_ = std::make_shared<swerve_controller::ParamListener>(get_node());
    // Only a committed parameter change can affect RT behavior. Validation remains generated.
    policy_callback_ = get_node()->add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "unsupported_command_policy") {
            const uint64_t ignore = parameter.as_string() == "ignore" ? 1 : 0;
            auto previous = policy_state_.load();
            while ((previous & 1) != ignore &&
            !policy_state_.compare_exchange_weak(previous,
            ((previous + 2) & ~uint64_t{1}) | ignore)) {}
          }
        }
      });
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameter value: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameter type: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (const rclcpp::ParameterTypeException & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameter type: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid controller parameters: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> publication_lock(publication_mutex_);
  try {
    params_ = param_listener_->get_params();
    std::set<std::string> names;
    for (const auto * group : {&params_.wheel_names, &params_.actuator_names}) {
      if (group->size() < 3 || group->size() != params_.wheel_names.size()) {
        throw std::invalid_argument(
            "At least three matching wheel and actuator names are required");
      }
      for (const auto & name : *group) {
        if (name.empty() || !names.insert(name).second) {
          throw std::invalid_argument("Joint names must be nonempty and unique across all modules");
        }
      }
    }
    const auto supported_states = [](const auto & interfaces, const std::string & required) {
        std::set<std::string> unique(interfaces.begin(), interfaces.end());
        return unique.size() == interfaces.size() && unique.count(required) != 0 &&
               std::all_of(interfaces.begin(), interfaces.end(), [](const auto & value) {
                   return value == "position" || value == "velocity";
        });
      };
    if (params_.wheel_command_interfaces != std::vector<std::string>{"velocity"} ||
      params_.actuator_command_interfaces != std::vector<std::string>{"position"} ||
      !supported_states(params_.wheel_state_interfaces, "velocity") ||
      !supported_states(params_.actuator_state_interfaces, "position") ||
      std::find(params_.actuator_state_interfaces.begin(), params_.actuator_state_interfaces.end(),
        "velocity") == params_.actuator_state_interfaces.end())
    {
      throw std::invalid_argument("Unsupported command/state interface configuration");
    }
    const size_t count = params_.wheel_names.size();
    for (const auto * values : {&params_.module_x, &params_.module_y, &params_.wheel_radii,
        &params_.wheel_offsets, &params_.steering_min, &params_.steering_max,
        &params_.steering_velocity_limits})
    {
      if (values->size() != count || !std::all_of(values->begin(), values->end(),
        [](double value) {return std::isfinite(value);}))
      {
        throw std::invalid_argument("Module arrays must be finite and match joint-name count");
      }
    }
    if (params_.steering_continuous.size() != count ||
      !std::isfinite(params_.cmd_vel_timeout) || !std::isfinite(params_.odom_publish_rate) ||
      !std::isfinite(params_.command_epsilon) ||
      !std::isfinite(params_.steering_feedback_tolerance) ||
      params_.odom_publish_rate <= 0.0 || params_.odom_publish_rate > 1000.0 ||
      params_.odom_frame_id.empty() || params_.base_frame_id.empty() ||
      params_.odom_frame_id == params_.base_frame_id)
    {
      throw std::invalid_argument("Invalid steering flags, tuning, or frame IDs");
    }
    // Cache geometry and intersect controller/URDF limits before any hardware loans exist.
    const auto & limits = get_hard_joint_limits();
    modules_.clear();
    modules_.reserve(count);
    feedback_.assign(count, ModuleState{});
    targets_.assign(count, ModuleTarget{});
    for (size_t i = 0; i < count; ++i) {
      ModuleGeometry module{params_.module_x[i], params_.module_y[i], params_.wheel_radii[i],
        params_.wheel_offsets[i], params_.steering_continuous[i], params_.steering_min[i],
        params_.steering_max[i], params_.steering_velocity_limits[i]};
      if (module.radius <= 0.0 || module.velocity_limit <= 0.0 ||
        (!module.continuous && module.minimum >= module.maximum))
      {
        throw std::invalid_argument("Invalid module radius, steering travel, or slew rate");
      }
      const auto found = limits.find(params_.actuator_names[i]);
      if (found != limits.end()) {
        const auto & limit = found->second;
        if (limit.has_position_limits) {
          if (module.continuous) {
            throw std::invalid_argument("Continuous steering conflicts with bounded URDF joint");
          }
          module.minimum = std::max(module.minimum, limit.min_position);
          module.maximum = std::min(module.maximum, limit.max_position);
        }
        if (limit.has_velocity_limits) {
          module.velocity_limit = std::min(module.velocity_limit, limit.max_velocity);
        }
      }
      if (!std::isfinite(module.velocity_limit) || module.velocity_limit <= 0.0 ||
        (!module.continuous && module.minimum >= module.maximum))
      {
        throw std::invalid_argument("Controller and URDF steering limits do not overlap");
      }
      modules_.push_back(module);
    }
    // Coincident pivots cannot reliably distinguish translation from yaw for all headings.
    const bool distinct = std::any_of(modules_.begin() + 1, modules_.end(),
        [this](const auto & module) {
          return module.x != modules_[0].x || module.y != modules_[0].y;
      });
    std::array<double, 3> velocity{};
    if (!distinct || !estimate_chassis_velocity(
        modules_.data(), feedback_.data(), count, velocity))
    {
      throw std::invalid_argument("Module geometry cannot resolve chassis motion");
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot configure controller: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  policy_state_.store((policy_state_.load() & ~uint64_t{1}) |
    (params_.unsupported_command_policy == "ignore" ? 1 : 0));
  timeout_ns_ = static_cast<int64_t>(params_.cmd_vel_timeout * 1e9);
  // Executor callbacks own message allocation. Only primitive values enter the RT mailbox.
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1),
    [this](const geometry_msgs::msg::Twist & msg) {
      const auto activation = activation_ns_.load();
      if (!active_.load()) {return;}
      Command command;
      command.activation_ns = activation;
      command.policy_revision = policy_state_.load();
      command.received_ns = steady_now();
      const double epsilon = params_.command_epsilon;
      const bool finite = std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
      std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
      std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
      if (finite) {
        const auto nonzero = [epsilon](double value) {
          return value != 0.0 && std::abs(value) >= epsilon;
        };
        command.vx = nonzero(msg.linear.x) ? msg.linear.x : 0.0;
        command.vy = nonzero(msg.linear.y) ? msg.linear.y : 0.0;
        command.omega = nonzero(msg.angular.z) ? msg.angular.z : 0.0;
        command.unsupported = nonzero(msg.linear.z) || nonzero(msg.angular.x) ||
        nonzero(msg.angular.y);
      }
      // Nonfinite input replaces the prior command with a stop under either policy.
      command_box_.set(command);
    });
  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
  publish_timer_ = get_node()->create_wall_timer(
    std::chrono::duration<double>(1.0 / params_.odom_publish_rate),
    [this]() {publish_telemetry();});
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < params_.wheel_names.size(); ++i) {
    config.names.push_back(params_.wheel_names[i] + "/velocity");
    config.names.push_back(params_.actuator_names[i] + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
{
  auto config = command_interface_configuration();
  // Steering rate is needed to compensate wheel motion about an offset steering pivot.
  for (const auto & name : params_.actuator_names) {
    config.names.push_back(name + "/velocity");
  }
  return config;
}

void SwerveController::clear_references()
{
  wheel_velocity_commands_.clear();
  actuator_position_commands_.clear();
  wheel_velocity_states_.clear();
  actuator_position_states_.clear();
  actuator_velocity_states_.clear();
}

bool SwerveController::stop_wheels()
{
  bool success = true;
  for (auto & wheel : wheel_velocity_commands_) {
    // Do not short-circuit: a failure on one joint must not prevent stopping the others.
    success = write_command(wheel.get(), 0.0) && success;
  }
  return success;
}

controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> publication_lock(publication_mutex_);
  active_.store(false);
  clear_references();
  // Bind in module order regardless of the order in which the manager loans interfaces.
  const auto bind = [this](auto & loans, auto & refs, const std::string & name) {
      const auto found = std::find_if(loans.begin(), loans.end(), [&](const auto & interface) {
            return interface.get_name() == name;
      });
      if (found == loans.end()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Missing required interface: %s", name.c_str());
        return false;
      }
      refs.push_back(std::ref(*found));
      return true;
    };
  for (size_t i = 0; i < modules_.size(); ++i) {
    if (!bind(command_interfaces_, wheel_velocity_commands_,
        params_.wheel_names[i] + "/velocity") ||
      !bind(state_interfaces_, wheel_velocity_states_, params_.wheel_names[i] + "/velocity") ||
      !bind(command_interfaces_, actuator_position_commands_,
        params_.actuator_names[i] + "/position") ||
      !bind(state_interfaces_, actuator_position_states_,
        params_.actuator_names[i] + "/position") ||
      !bind(state_interfaces_, actuator_velocity_states_, params_.actuator_names[i] + "/velocity"))
    {
      (void)stop_wheels();
      clear_references();
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  bool ready = stop_wheels();
  for (auto & state : wheel_velocity_states_) {
    const auto value = state.get().get_optional(1);
    ready = value && std::isfinite(*value) && ready;
  }
  for (auto & state : actuator_velocity_states_) {
    const auto value = state.get().get_optional(1);
    ready = value && std::isfinite(*value) && ready;
  }
  for (size_t i = 0; i < modules_.size(); ++i) {
    const auto value = actuator_position_states_[i].get().get_optional(1);
    if (!value || !std::isfinite(*value) ||
      !steering_in_range(modules_[i], *value, params_.steering_feedback_tolerance))
    {
      ready = false;
    } else {
      ready = write_command(actuator_position_commands_[i].get(),
          bounded_angle(modules_[i], *value)) && ready;
    }
  }
  if (!ready) {
    RCLCPP_ERROR(get_node()->get_logger(),
        "Cannot initialize safe commands from hardware feedback");
    clear_references();
    return controller_interface::CallbackReturn::ERROR;
  }
  // The timestamp gate rejects callbacks from previous activations without resetting a live mailbox.
  last_command_ = Command{};
  activation_ns_.store(steady_now());
  odom_pub_->on_activate();
  active_.store(true);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SwerveController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> publication_lock(publication_mutex_);
  active_.store(false);
  const bool stopped = stop_wheels();
  odom_pub_->on_deactivate();
  clear_references();
  return stopped ? controller_interface::CallbackReturn::SUCCESS :
         controller_interface::CallbackReturn::ERROR;
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!active_.load()) {return controller_interface::return_type::OK;}

  // Validate all feedback before any calculation or command write; optional reads never throw.
  for (size_t i = 0; i < modules_.size(); ++i) {
    const auto angle = actuator_position_states_[i].get().get_optional(1);
    const auto velocity = wheel_velocity_states_[i].get().get_optional(1);
    const auto rate = actuator_velocity_states_[i].get().get_optional(1);
    Fault fault = Fault::NONE;
    if (!angle || !velocity || !rate) {
      fault = Fault::FEEDBACK_UNAVAILABLE;
    } else if (!std::isfinite(*angle) || !std::isfinite(*velocity) || !std::isfinite(*rate)) {
      fault = Fault::FEEDBACK_NONFINITE;
    } else if (!steering_in_range(modules_[i], *angle, params_.steering_feedback_tolerance)) {
      fault = Fault::STEERING_RANGE;
    }
    if (fault != Fault::NONE) {
      // The executor timer reports faults even after controller manager deactivates us.
      pending_fault_.store(fault);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
    feedback_[i] = {*angle, *velocity, *rate};
  }
  if (const auto command = command_box_.try_get()) {last_command_ = *command;}
  const int64_t now = steady_now();
  const uint64_t policy = policy_state_.load();
  const bool fresh = last_command_.policy_revision == policy &&
    last_command_.activation_ns == activation_ns_.load() &&
    last_command_.received_ns > activation_ns_.load() &&
    now >= last_command_.received_ns && now - last_command_.received_ns < timeout_ns_;
  const bool accepted = fresh && ((policy & 1) || !last_command_.unsupported);
  const double vx = accepted ? last_command_.vx : 0.0;
  const double vy = accepted ? last_command_.vy : 0.0;
  const double omega = accepted ? last_command_.omega : 0.0;
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt < 0.0) {
    pending_fault_.store(Fault::TARGET_NONFINITE);
    (void)stop_wheels();
    return controller_interface::return_type::ERROR;
  }
  bool reachable = true;
  for (size_t i = 0; i < modules_.size(); ++i) {
    reachable = inverse_kinematics(modules_[i], feedback_[i], vx, vy, omega, dt,
        targets_[i]) && reachable;
  }
  for (size_t i = 0; i < modules_.size(); ++i) {
    if (!std::isfinite(targets_[i].angle) || !std::isfinite(targets_[i].wheel_rate)) {
      pending_fault_.store(Fault::TARGET_NONFINITE);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
  }
  if (!reachable) {
    // A command outside bounded travel is recoverable: hold every steering joint and stop.
    for (size_t i = 0; i < modules_.size(); ++i) {
      targets_[i] = {bounded_angle(modules_[i], feedback_[i].angle), 0.0};
    }
  }
  // Solve feedback before commanding motion so a solver failure cannot leave new drive targets.
  std::array<double, 3> velocity{};
  if (!estimate_chassis_velocity(modules_.data(), feedback_.data(), modules_.size(), velocity)) {
    pending_fault_.store(Fault::ODOMETRY_SOLVE);
    (void)stop_wheels();
    return controller_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < modules_.size(); ++i) {
    const bool steering_written = write_command(actuator_position_commands_[i].get(),
        targets_[i].angle);
    const bool wheel_written = write_command(wheel_velocity_commands_[i].get(),
        targets_[i].wheel_rate);
    if (!steering_written || !wheel_written) {
      pending_fault_.store(Fault::COMMAND_WRITE);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
  }
  const double robot_vx = velocity[0], robot_vy = velocity[1], robot_omega = velocity[2];

  // Integrate position over time period
  if (dt > 0.0) {
    odom_x_ += (robot_vx * std::cos(odom_yaw_) - robot_vy * std::sin(odom_yaw_)) * dt;
    odom_y_ += (robot_vx * std::sin(odom_yaw_) + robot_vy * std::cos(odom_yaw_)) * dt;
    odom_yaw_ += robot_omega * dt;
  }

  // This bounded snapshot is consumed by an executor timer; publishing never runs in update().
  (void)telemetry_box_.try_set(Telemetry{
      odom_x_, odom_y_, odom_yaw_, robot_vx, robot_vy, robot_omega,
      time.nanoseconds(), activation_ns_.load()});
  return controller_interface::return_type::OK;
}

void SwerveController::publish_telemetry()
{
  std::lock_guard<std::mutex> publication_lock(publication_mutex_);
  const auto fault = pending_fault_.exchange(Fault::NONE);
  if (fault != Fault::NONE) {
    const char * reason = "unknown fault";
    switch (fault) {
      case Fault::FEEDBACK_UNAVAILABLE: reason = "feedback interface unavailable"; break;
      case Fault::FEEDBACK_NONFINITE: reason = "nonfinite joint feedback"; break;
      case Fault::STEERING_RANGE: reason = "steering feedback exceeds travel tolerance"; break;
      case Fault::TARGET_NONFINITE: reason = "nonfinite wheel or steering target"; break;
      case Fault::COMMAND_WRITE: reason = "hardware command write failed"; break;
      case Fault::ODOMETRY_SOLVE: reason = "chassis velocity estimation failed"; break;
      case Fault::NONE: break;
    }
    RCLCPP_ERROR(get_node()->get_logger(), "Controller update stopped: %s", reason);
  }
  if (!active_.load()) {return;}
  const auto sample = telemetry_box_.try_get();
  if (!sample || sample->activation_ns != activation_ns_.load()) {return;}
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Time(sample->stamp_ns);
  odom.header.frame_id = params_.odom_frame_id;
  odom.child_frame_id = params_.base_frame_id;
  odom.pose.pose.position.x = sample->x;
  odom.pose.pose.position.y = sample->y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, sample->yaw);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  odom.twist.twist.linear.x = sample->vx;
  odom.twist.twist.linear.y = sample->vy;
  odom.twist.twist.angular.z = sample->omega;
  odom_pub_->publish(odom);
  if (params_.enable_odom_tf && active_.load()) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = sample->x;
    transform.transform.translation.y = sample->y;
    transform.transform.rotation = odom.pose.pose.orientation;
    odom_tf_broadcaster_->sendTransform(transform);
  }
}
}  // namespace swervebot_controllers

PLUGINLIB_EXPORT_CLASS(
  swervebot_controllers::SwerveController, controller_interface::ControllerInterface)
