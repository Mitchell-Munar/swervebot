#include "swervebot_controllers/swerve_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "pluginlib/class_list_macros.hpp"

namespace swervebot_controllers // Must match the namespace in the header file
{
    controller_interface::CallbackReturn SwerveController::on_init()
    {
        try {
            // Initialize the parameter listener
            param_listener_ = std::make_shared<swerve_controller::ParamListener>(get_node());

            // Get the initial parameters
            params_ = param_listener_->get_params();
        }
        catch (const std::exception & e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Error occurred while initializing SwerveController: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SwerveController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // If params are modified at runtime or on boot, fetch the latest validated parameters
        if (param_listener_->is_old(params_)) {
            params_ = param_listener_->get_params();
        }

        // Setup the cmd_vel subscriber
        cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                rt_cmd_vel_ptr_.writeFromNonRT(msg);
            }
        );

        // Setup odometry publisher and realtime wrapper
        odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        rt_odom_pub_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odom_pub_);

        // Setup TF broadcaster
        odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

        last_publish_time_ = get_node()->get_clock()->now();

        RCLCPP_INFO(get_node()->get_logger(), "SwerveController configured successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }
    

    controller_interface::InterfaceConfiguration SwerveController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        // Explicitly enforce exactly 4 modules for sizing safety
        if (params_.wheel_names.size() != 4 || params_.actuator_names.size() != 4) {
            return config; 
        }

        for (size_t i = 0; i < 4; ++i) {
            // Hardcode the interface type string directly to the joint name
            config.names.push_back(params_.wheel_names[i] + "/velocity");
            config.names.push_back(params_.actuator_names[i] + "/position");
        }
        return config;
    }

    controller_interface::InterfaceConfiguration SwerveController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        // Explicitly enforce exactly 4 modules for sizing safety
        if (params_.wheel_names.size() != 4 || params_.actuator_names.size() != 4) {
            return config;
        }

        for (size_t i = 0; i < 4; ++i) {
            config.names.push_back(params_.wheel_names[i] + "/velocity");
            config.names.push_back(params_.actuator_names[i] + "/position");
        }
        return config;
    }

    controller_interface::CallbackReturn SwerveController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Clear out local cache vectors from previous runs to prevent duplicate references
        wheel_velocity_commands_.clear();
        wheel_velocity_states_.clear();
        actuator_position_commands_.clear();
        actuator_position_states_.clear();

        if (params_.wheel_names.size() != 4 || params_.actuator_names.size() != 4) {
            RCLCPP_ERROR(get_node()->get_logger(), "SwerveController requires exactly 4 wheel and 4 actuator names!");
            return controller_interface::CallbackReturn::ERROR;
        }

        for (size_t i = 0; i < 4; ++i) {
            // Grab Wheel Velocity Command
            auto wheel_cmd_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](const auto & interface) {
                    return interface.get_prefix_name() == params_.wheel_names[i] && interface.get_interface_name() == "velocity";
                });
            if (wheel_cmd_handle == command_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
            wheel_velocity_commands_.push_back(std::ref(*wheel_cmd_handle));

            // Grab Wheel Velocity State
            auto wheel_state_handle = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & interface) {
                    return interface.get_prefix_name() == params_.wheel_names[i] && interface.get_interface_name() == "velocity";
                });
            if (wheel_state_handle == state_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
            wheel_velocity_states_.push_back(std::ref(*wheel_state_handle));

            // Grab Actuator Position Command
            auto act_cmd_handle = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                [&](const auto & interface) {
                    return interface.get_prefix_name() == params_.actuator_names[i] && interface.get_interface_name() == "position";
                });
            if (act_cmd_handle == command_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
            actuator_position_commands_.push_back(std::ref(*act_cmd_handle));

            // Grab Actuator Position State
            auto act_state_handle = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                [&](const auto & interface) {
                    return interface.get_prefix_name() == params_.actuator_names[i] && interface.get_interface_name() == "position";
                });
            if (act_state_handle == state_interfaces_.end()) return controller_interface::CallbackReturn::ERROR;
            actuator_position_states_.push_back(std::ref(*act_state_handle));
        }
        RCLCPP_INFO(get_node()->get_logger(), "SwerveController activated successfully.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SwerveController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Release your references to the hardware loan tracking arrays
        wheel_velocity_commands_.clear();
        wheel_velocity_states_.clear();
        actuator_position_commands_.clear();
        actuator_position_states_.clear();

        RCLCPP_INFO(get_node()->get_logger(), "Swerve controller successfully deactivated.");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    void SwerveController::optimizeSteeringAngle(double &target_angle, double &target_velocity, double /*current_angle*/)
    {
        // 1. Normalize the target angle to be between -pi and pi
        target_angle = angles::normalize_angle(target_angle);

        // 2. Check if the target angle falls outside the physical 180-degree window [-pi/2, pi/2]
        if (target_angle > M_PI_2)
        {
            // Pointing backwards-left: Flip it into the front window and reverse wheel
            target_angle -= M_PI;
            target_velocity = -target_velocity;
        }
        else if (target_angle < -M_PI_2)
        {
            // Pointing backwards-right: Flip it into the front window and reverse wheel
            target_angle += M_PI;
            target_velocity = -target_velocity;
        }
        
        // Now target_angle is strictly guaranteed to be between -pi/2 and pi/2 (-90° and +90°)
    }

    controller_interface::return_type SwerveController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        if (param_listener_->is_old(params_)) {
            params_ = param_listener_->get_params();
        }

        // Read latest command twist safely
        auto cmd_ptr = rt_cmd_vel_ptr_.readFromRT();
        geometry_msgs::msg::Twist cmd;
        if (cmd_ptr && *cmd_ptr) {
            cmd = **cmd_ptr;
        }

        double vx = cmd.linear.x;
        double vy = cmd.linear.y;
        double omega = cmd.angular.z;

        // Module positions relative to robot center (FL, FR, BL, BR)
        double l = params_.wheel_base;
        double w = params_.track_width;
        double r_x[4] = { l / 2.0,  l / 2.0, -l / 2.0, -l / 2.0};
        double r_y[4] = { w / 2.0, -w / 2.0,  w / 2.0, -w / 2.0};

        double target_angles[4];
        double target_velocities[4];

        for (size_t i = 0; i < 4; ++i)
        {
            double v_xi = 0.0;
            double v_yi = 0.0;

            switch (current_mode_)
            {
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

            if (current_mode_ == SwerveMode::CRABWALK)
            {
                target_angles[i] = std::atan2(vy, vx);
                target_velocities[i] = std::hypot(vx, vy) / params_.wheel_radius;
            }
            else if (current_mode_ == SwerveMode::ZERO_TURN)
            {
                target_angles[i] = std::atan2(r_y[i], r_x[i]) + M_PI_2;
                target_velocities[i] = omega * std::hypot(r_x[i], r_y[i]) / params_.wheel_radius;
            }
            else
            {
                target_angles[i] = std::atan2(v_yi, v_xi);
                target_velocities[i] = std::hypot(v_xi, v_yi) / params_.wheel_radius;
            }

            // Get current steering angle from state reference vector
            double current_steer_pos = actuator_position_states_[i].get().get_optional().value();
            optimizeSteeringAngle(target_angles[i], target_velocities[i], current_steer_pos);

            // Write commands to hardware interface references
            const bool actuator_written =
                actuator_position_commands_[i].get().set_value(target_angles[i]);

            const bool wheel_written =
                wheel_velocity_commands_[i].get().set_value(target_velocities[i]);

            if (!actuator_written || !wheel_written)
            {
                RCLCPP_ERROR(
                    get_node()->get_logger(),
                    "Failed to write commands for swerve module %zu",
                    i);
            }
        }

        // ==========================================
        // Odometry & Forward Kinematics Integration
        // ==========================================
        double sum_vxi = 0.0;
        double sum_vyi = 0.0;
        double sum_cross_prod = 0.0;
        double sum_r_squared = 0.0;

        for (size_t i = 0; i < 4; ++i)
        {
            double current_steer_pos = actuator_position_states_[i].get().get_optional().value();
            double current_wheel_vel = wheel_velocity_states_[i].get().get_optional().value();
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
        if (dt > 0.0)
        {
            odom_x_ += (robot_vx * std::cos(odom_yaw_) - robot_vy * std::sin(odom_yaw_)) * dt;
            odom_y_ += (robot_vx * std::sin(odom_yaw_) + robot_vy * std::cos(odom_yaw_)) * dt;
            odom_yaw_ += robot_omega * dt;
        }

        // Publish real-time odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = params_.odom_frame_id;
        odom_msg.child_frame_id = params_.base_frame_id;

        odom_msg.pose.pose.position.x = odom_x_;
        odom_msg.pose.pose.position.y = odom_y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, odom_yaw_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = robot_vx;
        odom_msg.twist.twist.linear.y = robot_vy;
        odom_msg.twist.twist.angular.z = robot_omega;

        rt_odom_pub_->try_publish(odom_msg);

        // Broadcast TF transform if enabled
        if (params_.enable_odom_tf)
        {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = time;
            tf_msg.header.frame_id = params_.odom_frame_id;
            tf_msg.child_frame_id = params_.base_frame_id;
            tf_msg.transform.translation.x = odom_x_;
            tf_msg.transform.translation.y = odom_y_;
            tf_msg.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, odom_yaw_);
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            odom_tf_broadcaster_->sendTransform(tf_msg);
        }

        return controller_interface::return_type::OK;
    }
} // namespace swervebot_controllers

PLUGINLIB_EXPORT_CLASS(
  swervebot_controllers::SwerveController,
  controller_interface::ControllerInterface
)