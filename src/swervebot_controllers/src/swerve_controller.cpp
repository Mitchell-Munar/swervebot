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
      !supported_states(params_.actuator_state_interfaces, "position"))
    {
      throw std::invalid_argument("Unsupported command/state interface configuration");
    }
    if (!std::isfinite(params_.wheel_base) || !std::isfinite(params_.track_width) ||
      !std::isfinite(params_.wheel_radius) || !std::isfinite(params_.cmd_vel_timeout) ||
      params_.cmd_vel_timeout > 3600.0 || params_.odom_frame_id.empty() ||
      params_.base_frame_id.empty())
    {
      throw std::invalid_argument("Geometry, timeout and frame IDs must be finite and valid");
    }
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot configure controller: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  timeout_ns_ = static_cast<int64_t>(params_.cmd_vel_timeout * 1e9);
  // Executor callbacks own message allocation. Only primitive values enter the RT mailbox.
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1), [this](const geometry_msgs::msg::Twist & msg) {
      const auto activation = activation_ns_.load();
      if (!active_.load()) {return;}
      Command command;
      command.activation_ns = activation;
      command.received_ns = steady_now();
      if (std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
      std::isfinite(msg.angular.z))
      {
        command.vx = msg.linear.x;
        command.vy = msg.linear.y;
        command.omega = msg.angular.z;
      }
      // Invalid input replaces the previous command with a stop, never a stale moving command.
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
  // Only the feedback used by update() is claimed. The broadcaster can also read these states.
  return command_interface_configuration();
}

void SwerveController::clear_references()
{
  wheel_velocity_commands_.clear();
  actuator_position_commands_.clear();
  wheel_velocity_states_.clear();
  actuator_position_states_.clear();
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
      !bind(state_interfaces_, actuator_position_states_, params_.actuator_names[i] + "/position"))
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
  for (size_t i = 0; i < 4; ++i) {
    const auto value = actuator_position_states_[i].get().get_optional(1);
    if (!value || !std::isfinite(*value) || std::abs(*value) > M_PI_2) {
      ready = false;
    } else {
      ready = write_command(actuator_position_commands_[i].get(), *value) && ready;
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
  std::array<double, 4> steering{}, wheels{};
  // Validate all feedback before any calculation or command write; optional reads never throw.
  for (size_t i = 0; i < 4; ++i) {
    const auto angle = actuator_position_states_[i].get().get_optional(1);
    const auto velocity = wheel_velocity_states_[i].get().get_optional(1);
    if (!angle || !velocity || !std::isfinite(*angle) || !std::isfinite(*velocity) ||
      std::abs(*angle) > M_PI_2)
    {
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
    steering[i] = *angle;
    wheels[i] = *velocity;
  }
  if (const auto command = command_box_.try_get()) {last_command_ = *command;}
  const int64_t now = steady_now();
  const bool fresh = last_command_.activation_ns == activation_ns_.load() &&
    last_command_.received_ns > activation_ns_.load() &&
    now >= last_command_.received_ns && now - last_command_.received_ns < timeout_ns_;
  const double vx = fresh ? last_command_.vx : 0.0;
  const double vy = fresh ? last_command_.vy : 0.0;
  const double omega = fresh ? last_command_.omega : 0.0;
  // Module positions relative to robot center (FL, FR, BL, BR)
  double l = params_.wheel_base;
  double w = params_.track_width;
  double r_x[4] = {l / 2.0, l / 2.0, -l / 2.0, -l / 2.0};
  double r_y[4] = {w / 2.0, -w / 2.0, w / 2.0, -w / 2.0};

  double target_angles[4];
  double target_velocities[4];

  for (size_t i = 0; i < 4; ++i) {
    double v_xi = 0.0;
    double v_yi = 0.0;

    // Chassis translation plus omega cross module-offset gives each wheel velocity.
    switch (current_mode_) {
      case SwerveMode::NORMAL:
        v_xi = vx - omega * r_y[i];
        v_yi = vy + omega * r_x[i];
        break;
      case SwerveMode::CRABWALK:
        v_xi = vx;
        v_yi = vy;
        break;
      case SwerveMode::ZERO_TURN:
        v_xi = -omega * r_y[i];
        v_yi = omega * r_x[i];
        break;
    }

    // atan2 chooses steering direction; dividing linear speed by radius gives rad/s.
    if (current_mode_ == SwerveMode::CRABWALK) {
      target_angles[i] = std::atan2(vy, vx);
      target_velocities[i] = std::hypot(vx, vy) / params_.wheel_radius;
    } else if (current_mode_ == SwerveMode::ZERO_TURN) {
      target_angles[i] = std::atan2(r_y[i], r_x[i]) + M_PI_2;
      target_velocities[i] = omega * std::hypot(r_x[i], r_y[i]) / params_.wheel_radius;
    } else {
      target_angles[i] = std::atan2(v_yi, v_xi);
      target_velocities[i] = std::hypot(v_xi, v_yi) / params_.wheel_radius;
    }

    // Get current steering angle from state reference vector
    double current_steer_pos = steering[i];
    optimizeSteeringAngle(target_angles[i], target_velocities[i], current_steer_pos);

    // A stopped chassis holds the measured steering position instead of snapping to zero.
    if (vx == 0.0 && vy == 0.0 && omega == 0.0) {
      target_angles[i] = steering[i];
    }
    if (!std::isfinite(target_angles[i]) || !std::isfinite(target_velocities[i])) {
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
      (void)stop_wheels();
      return controller_interface::return_type::ERROR;
    }
  }

  // Resolve measured wheel velocities into chassis velocity for odometry.
  double sum_vxi = 0.0;
  double sum_vyi = 0.0;
  double sum_cross_prod = 0.0;
  double sum_r_squared = 0.0;

  for (size_t i = 0; i < 4; ++i) {
    double current_steer_pos = steering[i];
    double current_wheel_vel = wheels[i];
    double linear_wheel_speed = current_wheel_vel * params_.wheel_radius;

    double v_xi = linear_wheel_speed * std::cos(current_steer_pos);
    double v_yi = linear_wheel_speed * std::sin(current_steer_pos);

    sum_vxi += v_xi;
    sum_vyi += v_yi;

    sum_cross_prod += (v_yi * r_x[i] - v_xi * r_y[i]);
    sum_r_squared += (r_x[i] * r_x[i] + r_y[i] * r_y[i]);
  }

  // Least-squares forward kinematics calculation
  double robot_vx = sum_vxi / 4.0;
  double robot_vy = sum_vyi / 4.0;
  double robot_omega = (sum_r_squared > 0.0) ? (sum_cross_prod / sum_r_squared) : 0.0;

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
