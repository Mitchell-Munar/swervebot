import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory

class TeleopNode(Node):
    # Drive mode integer mapping constants
    MODE_ACKERMANN = 0
    MODE_CRAB_WALK = 1
    MODE_ZERO_TURN = 2

    def __init__(self):
        super().__init__('teleop_node')

        # 1. Declare a parameter for custom config overrides
        self.declare_parameter('config_file', '')
        custom_config_path = self.get_parameter('config_file').value

        # 2. Determine path: Use custom path if provided, otherwise fallback to default bundled config
        if custom_config_path and os.path.exists(custom_config_path):
            config_path = custom_config_path
        else:
            pkg_share = get_package_share_directory('swervebot_teleop')
            config_path = os.path.join(pkg_share, 'config', 'game_controller.yaml')

        # 3. Load the YAML file
        self.get_logger().info(f'Loading teleop configuration from: {config_path}')
        self.load_config(config_path)

        # 4. State tracking for drive modes (default to Ackermann = 0)
        self.drive_mode = self.MODE_ACKERMANN
        self.last_button_states = {}

        # 5. ROS 2 Publishers & Subscribers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel/teleop', 10)
        self.mode_publisher_ = self.create_publisher(Int32, '/swervebot/drive_mode', 10)
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

    def load_config(self, path):
        try:
            with open(path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.axes_map = self.config.get('axes', {})
            self.buttons_map = self.config.get('buttons', {})
            self.settings = self.config.get('settings', {})
            self.deadzone = self.settings.get('deadzone', 0.05)
            self.linear_scale = self.settings.get('linear_scale_max', 1.5)
            self.angular_scale = self.settings.get('angular_scale_max', 1.0)
        except Exception as e:
            self.get_logger().error(f'Failed to load YAML config: {e}')
            self.config = {}
            self.axes_map, self.buttons_map, self.settings = {}, {}, {}
            self.deadzone, self.linear_scale, self.angular_scale = 0.05, 1.5, 1.0

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg: Joy):
        btn_ackermann = self.buttons_map.get('toggle_ackermann', -1)
        btn_crab = self.buttons_map.get('toggle_crab_walk', -1)
        btn_hold_zero = self.buttons_map.get('hold_zero_turn', -1)

        # Handle Hold mode for Zero Turn (Overrides current mode while pressed)
        is_holding_zero = False
        if btn_hold_zero != -1 and len(msg.buttons) > btn_hold_zero:
            if msg.buttons[btn_hold_zero] == 1:
                is_holding_zero = True

        # Handle Toggle states on rising edge (0 -> 1 transition)
        if not is_holding_zero:
            if btn_ackermann != -1 and len(msg.buttons) > btn_ackermann:
                pressed = msg.buttons[btn_ackermann]
                last_state = self.last_button_states.get(btn_ackermann, 0)
                if pressed == 1 and last_state == 0:
                    self.drive_mode = self.MODE_ACKERMANN
                    self.get_logger().info('Drive Mode Switched: Ackermann (0)')
                self.last_button_states[btn_ackermann] = pressed

            if btn_crab != -1 and len(msg.buttons) > btn_crab:
                pressed = msg.buttons[btn_crab]
                last_state = self.last_button_states.get(btn_crab, 0)
                if pressed == 1 and last_state == 0:
                    self.drive_mode = self.MODE_CRAB_WALK
                    self.get_logger().info('Drive Mode Switched: Crab Walk (1)')
                self.last_button_states[btn_crab] = pressed

        # Active drive mode evaluation
        active_mode = self.MODE_ZERO_TURN if is_holding_zero else self.drive_mode

        # Parse Axes values with deadzones
        throttle_idx = self.axes_map.get('throttle', -1)
        steering_idx = self.axes_map.get('steering', -1)

        raw_throttle = msg.axes[throttle_idx] if throttle_idx != -1 and len(msg.axes) > throttle_idx else 0.0
        raw_steering = msg.axes[steering_idx] if steering_idx != -1 and len(msg.axes) > steering_idx else 0.0

        throttle = self.apply_deadzone(raw_throttle) * self.linear_scale
        steering = self.apply_deadzone(raw_steering) * self.angular_scale

        # Construct and Publish Twist message
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = steering
        self.publisher_.publish(twist)

        # Publish integer drive mode for the controller
        mode_msg = Int32()
        mode_msg.data = active_mode
        self.mode_publisher_.publish(mode_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()