#ifndef SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_
#define SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <std_msgs/msg/bool.hpp>

#include "controller_interface/controller_interface.hpp"
#include "swervebot_controllers/swerve_controller_parameters.hpp"

#include "tf2_ros/transform_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace swervebot_controllers // Name of your package
{
    enum class SwerveMode
    {
        NORMAL = 0,
        CRABWALK = 1,
        ZERO_TURN = 2
    };

    class SwerveController : public controller_interface::ControllerInterface
    {
        public:
            SwerveController() = default;

            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;
            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
            void optimizeSteeringAngle(double &target_angle, double &target_velocity, double current_angle);

        private:
            // Instantiating the auto-generated parameters struct
            std::shared_ptr<swerve_controller::ParamListener> param_listener_;
            swerve_controller::Params params_;

            // Controller state and mode
            SwerveMode current_mode_ = SwerveMode::NORMAL;

            // Hardware interface reference wrappers
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> wheel_velocity_commands_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> wheel_velocity_states_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> actuator_position_commands_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> actuator_position_states_;

            // Drive command subscription
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> rt_cmd_vel_ptr_;
            rclcpp::Time last_cmd_time_;

            // Drive mode command subscription
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_mode_sub_;
            realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Bool>> rt_drive_mode_ptr_;

            // Odometry publishing & TF broadcaster
            rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
            std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> rt_odom_pub_;
            std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

            // Odometry states
            rclcpp::Time last_publish_time_;
            double odom_x_ = 0.0;
            double odom_y_ = 0.0;
            double odom_yaw_ = 0.0;
    };
} // namespace swervebot_controllers

#endif // SWERVEBOT_CONTROLLERS__SWERVE_CONTROLLER_HPP_