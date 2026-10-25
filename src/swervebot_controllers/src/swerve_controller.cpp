#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

#include "angles/angles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "swervebot_controllers/swerve_controller.hpp"
#include "swervebot_controllers/swerve_kinematics.hpp"

namespace swervebot_controllers
{
namespace
{
// Permit small measured overshoot at a hard stop (0.057 degrees), never out-of-range commands.
constexpr double steering_feedback_tolerance = 1e-3;

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
      if (group->size() != 4) {
        throw std::invalid_argument("Exactly four wheel and four actuator names are required");
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
    if (!std::isfinite(params_.wheel_base) || !std::isfinite(params_.track_width) ||
      !std::isfinite(params_.wheel_radius) || !std::isfinite(params_.wheel_offset) ||
      !std::isfinite(params_.cmd_vel_timeout) || !std::isfinite(params_.max_steering_velocity) ||
      params_.cmd_vel_timeout > 3600.0 || params_.odom_frame_id.empty() ||
      params_.base_frame_id.empty())
    {
      throw std::invalid_argument("Geometry, timeout and frame IDs must be finite and valid");
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot configure controller: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  // Cache joint limits outside update(); the tighter controller/URDF rate wins.
  const auto & joint_limits = get_hard_joint_limits();
  for (size_t i = 0; i < 4; ++i) {
    steering_velocity_limits_[i] = params_.max_steering_velocity;
    const auto found = joint_limits.find(params_.actuator_names[i]);
    if (found != joint_limits.end() && found->second.has_velocity_limits) {
      steering_velocity_limits_[i] = std::min(
        steering_velocity_limits_[i], found->second.max_velocity);
    }
  }
  policy_state_.store((policy_state_.load() & ~uint64_t{1}) |
    (params_.unsupported_command_policy == "ignore" ? 1 : 0));
  timeout_ns_ = static_cast<int64_t>(params_.cmd_vel_timeout * 1e9);
  // Executor callbacks own message allocation. Only primitive values enter the RT mailbox.
  cmd_vel_sub_ = get_node()->create_subscription<swervebot_msgs::msg::RobotCommand>(
    "/swervebot/cmd_raw", rclcpp::QoS(1),
    [this](const swervebot_msgs::msg::RobotCommand & message) {
      using RobotCommand = swervebot_msgs::msg::RobotCommand;
      const auto & msg = message.twist;
      const auto activation = activation_ns_.load();
      if (!active_.load()) {return;}
      Command command;
      command.activation_ns = activation;
      command.policy_revision = policy_state_.load();
      command.received_ns = steady_now();
      constexpr double epsilon = 1e-6;
      const bool finite = std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
      std::isfinite(msg.linear.z) && std::isfinite(msg.angular.x) &&
      std::isfinite(msg.angular.y) && std::isfinite(msg.angular.z);
      if (finite && message.mode <= RobotCommand::ZERO_TURN) {
        command.mode = message.mode;
        command.vx = std::abs(msg.linear.x) < epsilon ? 0.0 : msg.linear.x;
        command.omega = std::abs(msg.angular.z) < epsilon ? 0.0 : msg.angular.z;
        command.vy = std::abs(msg.linear.y) < epsilon ? 0.0 : msg.linear.y;
        // Project planar motion into the selected mode before applying unsupported-field policy.
        if (command.mode == RobotCommand::CRAB_WALK) {
          command.omega = 0.0;
        } else if (command.mode == RobotCommand::ZERO_TURN) {
          command.vx = command.vy = 0.0;
        }
        command.unsupported = std::abs(msg.linear.z) >= epsilon ||
        std::abs(msg.angular.x) >= epsilon ||
        std::abs(msg.angular.y) >= epsilon;
        if (command.mode == RobotCommand::ACKERMANN) {
          command.unsupported = command.unsupported || command.vy != 0.0 ||
          (command.vx == 0.0 && command.omega != 0.0);
          command.vy = 0.0;
          if (command.vx == 0.0) {command.omega = 0.0;}
        }
      }
      // Invalid mode or nonfinite input always replaces the previous command with a stop, in either policy.
      command_box_.set(command);
    });
  odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());
  publish_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(20), [this]() {publish_telemetry();});
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
  for (size_t i = 0; i < 4; ++i) {
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
  for (size_t i = 0; i < 4; ++i) {
    const auto value = actuator_position_states_[i].get().get_optional(1);
    if (!value || !std::isfinite(*value) ||
      std::abs(*value) > M_PI_2 + steering_feedback_tolerance)
    {
      ready = false;
    } else {
      ready = write_command(actuator_position_commands_[i].get(),
          std::clamp(*value, -M_PI_2, M_PI_2)) && ready;
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

void SwerveController::optimizeSteeringAngle(
  double & target_angle, double & target_velocity,
  double /*current_angle*/)
{
  // 1. Normalize the target angle to be between -pi and pi
  target_angle = angles::normalize_angle(target_angle);

  // 2. Check if the target angle falls outside the physical 180-degree window [-pi/2, pi/2]
  if (target_angle > M_PI_2) {
    // Pointing backwards-left: Flip it into the front window and reverse wheel
    target_angle -= M_PI;
    target_velocity = -target_velocity;
  } else if (target_angle < -M_PI_2) {
    // Pointing backwards-right: Flip it into the front window and reverse wheel
    target_angle += M_PI;
    target_velocity = -target_velocity;
  }

  // Now target_angle is strictly guaranteed to be between -pi/2 and pi/2 (-90° and +90°)
}

controller_interface::return_type SwerveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!active_.load()) {return controller_interface::return_type::OK;}
  std::array<double, 4> steering{}, wheels{}, steering_rates{};
  // Validate all feedback before any calculation or command write; optional reads never throw.
  for (size_t i = 0; i < 4; ++i) {
    const auto angle = actuator_position_states_[i].get().get_optional(1);
    const auto velocity = wheel_velocity_states_[i].get().get_optional(1);
    const auto rate = actuator_velocity_states_[i].get().get_optional(1);
    Fault fault = Fault::NONE;
    if (!angle || !velocity || !rate) {
      fault = Fault::FEEDBACK_UNAVAILABLE;
    } else if (!std::isfinite(*angle) || !std::isfinite(*velocity) || !std::isfinite(*rate)) {
      fault = Fault::FEEDBACK_NONFINITE;
    } else if (std::abs(*angle) > M_PI_2 + steering_feedback_tolerance) {
      fault = Fault::STEERING_RANGE;
    }
    if (fault != Fault::NONE) {
      // The executor timer reports faults even after controller manager deactivates us.
      pending_fault_.store(fault);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
    steering[i] = *angle;
    wheels[i] = *velocity;
    steering_rates[i] = *rate;
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
  // Module positions relative to robot center (FL, FR, BL, BR)
  double l = params_.wheel_base;
  double w = params_.track_width;
  double r_x[4] = {l / 2.0, l / 2.0, -l / 2.0, -l / 2.0};
  double r_y[4] = {w / 2.0, -w / 2.0, w / 2.0, -w / 2.0};

  double target_angles[4];
  double target_velocities[4];

  for (size_t i = 0; i < 4; ++i) {
    // Body axes are x forward, y left, z up. v_pivot = v_body + omega cross r_pivot.
    const double pivot_x = vx - omega * r_y[i];
    const double pivot_y = vy + omega * r_x[i];
    target_angles[i] = std::atan2(pivot_y, pivot_x);
    double unused_speed = 0.0;
    optimizeSteeringAngle(target_angles[i], unused_speed, steering[i]);
    // Bound each position step by rate*dt, rather than relying on simulator command clipping.
    // A nanoradian of headroom avoids simulator clipping from floating-point boundary rounding.
    const double step = std::max(
      steering_velocity_limits_[i] * std::max(period.seconds(), 0.0) - 1e-9, 0.0);
    target_angles[i] = std::clamp(target_angles[i], steering[i] - step, steering[i] + step);

    // The wheel center is offset along local y: left +offset, right -offset.
    // Its rolling velocity includes -offset*(body yaw rate + measured steering rate).
    // Project onto the actual wheel direction while steering catches up to reduce scrub.
    const double offset = (i % 2 == 0 ? 1.0 : -1.0) * params_.wheel_offset;
    target_velocities[i] = (pivot_x * std::cos(steering[i]) +
      pivot_y * std::sin(steering[i]) - offset * (omega + steering_rates[i])) /
      params_.wheel_radius;
    if (vx == 0.0 && vy == 0.0 && omega == 0.0) {
      target_angles[i] = steering[i];
      target_velocities[i] = 0.0;
    }
    // Feedback tolerance does not widen the physical steering command window.
    target_angles[i] = std::clamp(target_angles[i], -M_PI_2, M_PI_2);
    if (!std::isfinite(target_angles[i]) || !std::isfinite(target_velocities[i])) {
      pending_fault_.store(Fault::TARGET_NONFINITE);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
  }
  // Compute every target before writing any command; on failure attempt to stop all wheels.
  for (size_t i = 0; i < 4; ++i) {
    const bool steering_written = write_command(actuator_position_commands_[i].get(),
        target_angles[i]);
    const bool wheel_written = write_command(wheel_velocity_commands_[i].get(),
        target_velocities[i]);
    if (!steering_written || !wheel_written) {
      pending_fault_.store(Fault::COMMAND_WRITE);
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
  }

  // Solve the measured contact constraints, including the same lateral offsets as IK.
  std::array<double, 3> velocity{};
  if (!estimate_chassis_velocity(r_x, r_y, params_.wheel_radius, params_.wheel_offset,
      steering, wheels, steering_rates, velocity))
  {
    pending_fault_.store(Fault::ODOMETRY_SOLVE);
    (void)stop_wheels();
    return controller_interface::return_type::ERROR;
  }
  const double robot_vx = velocity[0], robot_vy = velocity[1], robot_omega = velocity[2];

  // Integrate position over time period
  double dt = period.seconds();
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
