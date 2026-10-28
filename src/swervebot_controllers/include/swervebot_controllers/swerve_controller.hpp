#ifndef SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_
#define SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_thread_safe_box.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "swervebot_controllers/swerve_controller_parameters.hpp"
#include "swervebot_controllers/swerve_kinematics.hpp"

namespace swervebot_controllers
{
class SwerveController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Only fixed-size values cross the executor/update boundary. No ROS messages or shared pointers.
  struct Command
  {
    double vx = 0.0, vy = 0.0, omega = 0.0;
    bool unsupported = false;
    uint64_t policy_revision = 0;
    int64_t received_ns = 0, activation_ns = 0;
  };
  struct Telemetry
  {
    double x = 0.0, y = 0.0, yaw = 0.0;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    int64_t stamp_ns = 0, activation_ns = 0;
  };
  enum class Fault : uint8_t
  {
    NONE, FEEDBACK_UNAVAILABLE, FEEDBACK_NONFINITE, STEERING_RANGE,
    TARGET_NONFINITE, COMMAND_WRITE, ODOMETRY_SOLVE
  };
  std::atomic<Fault> pending_fault_{Fault::NONE};
  static int64_t steady_now();
  void clear_references();
  bool stop_wheels();
  void publish_telemetry();

  std::shared_ptr<swerve_controller::ParamListener> param_listener_;
  swerve_controller::Params params_;
  // Bit 0 selects ignore/stop; the remaining bits invalidate commands after policy changes.
  std::atomic<uint64_t> policy_state_{0};
  rclcpp_lifecycle::LifecycleNode::PostSetParametersCallbackHandle::SharedPtr policy_callback_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  wheel_velocity_commands_, actuator_position_commands_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  wheel_velocity_states_, actuator_position_states_, actuator_velocity_states_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  realtime_tools::RealtimeThreadSafeBox<Command> command_box_;
  realtime_tools::RealtimeThreadSafeBox<Telemetry> telemetry_box_;
  Command last_command_;
  std::atomic<bool> active_{false};
  std::atomic<int64_t> activation_ns_{0};
  int64_t timeout_ns_ = 0;
  // Sized once in on_configure; update only overwrites existing elements.
  std::vector<ModuleGeometry> modules_;
  std::vector<ModuleState> feedback_;
  std::vector<ModuleTarget> targets_;

  // Serializes executor publication with lifecycle publisher/configuration changes, never update().
  std::mutex publication_mutex_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  double odom_x_ = 0.0, odom_y_ = 0.0, odom_yaw_ = 0.0;
};
}  // namespace swervebot_controllers

#endif  // SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_
