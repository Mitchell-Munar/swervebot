"""Project joystick inputs into manual drive modes and publish standard chassis Twist."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml  # type: ignore[import-untyped]


MODE_ACKERMANN = 0
MODE_CRAB_WALK = 1
MODE_ZERO_TURN = 2


def project_command(mode: int, throttle: float, steering: float) -> Twist:
    """Keep manual mode restrictions outside the reusable chassis controller."""
    command = Twist()
    if not math.isfinite(throttle) or not math.isfinite(steering):
        return command
    if mode == MODE_ACKERMANN:
        command.linear.x = throttle
        # Ackermann requires forward/reverse motion; yaw-only input must remain stopped.
        command.angular.z = steering if throttle != 0.0 else 0.0
    elif mode == MODE_CRAB_WALK:
        command.linear.x = throttle
        command.linear.y = steering
    elif mode == MODE_ZERO_TURN:
        command.angular.z = steering
    return command


class TeleopNode(Node):
    """Own button transitions and joystick scaling; hardware receives only chassis motion."""

    def __init__(self):
        super().__init__('teleop_node')
        self.declare_parameter('config_file', '')
        share = Path(get_package_share_directory('swervebot_teleop'))
        default = share / 'config/game_controller.yaml'
        custom = self.get_parameter('config_file').value
        self.load_config(Path(custom) if custom else default)
        self.drive_mode = MODE_ACKERMANN
        self.last_button_states = {}
        self.command_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

    def load_config(self, path):
        """Reject malformed profiles explicitly rather than silently selecting another file."""
        try:
            with path.open(encoding='utf-8') as stream:
                config = yaml.safe_load(stream)
            if not isinstance(config, dict):
                raise ValueError('Profile must be a mapping')
            self.axes_map = config.get('axes', {})
            self.buttons_map = config.get('buttons', {})
            settings = config.get('settings', {})
            if not all(isinstance(value, dict) for value in
                       (self.axes_map, self.buttons_map, settings)):
                raise ValueError('axes, buttons and settings must be mappings')
            if not all(isinstance(value, int) and value >= -1
                       for value in (*self.axes_map.values(), *self.buttons_map.values())):
                raise ValueError('Axis/button indices must be integers >= -1')
            self.deadzone = float(settings.get('deadzone', 0.05))
            self.linear_scale = float(settings.get('linear_scale_max', 1.5))
            self.angular_scale = float(settings.get('angular_scale_max', 1.0))
            if (not all(math.isfinite(value) for value in
                        (self.deadzone, self.linear_scale, self.angular_scale)) or
                    not 0.0 <= self.deadzone < 1.0 or
                    min(self.linear_scale, self.angular_scale) < 0.0):
                raise ValueError('Invalid deadzone or velocity scales')
        except (OSError, yaml.YAMLError, TypeError, ValueError) as error:
            self.get_logger().error(f'Cannot load joystick profile {path}: {error}')
            raise ValueError(f'Invalid joystick profile: {path}') from error

    def apply_deadzone(self, value):
        """Rescale the usable stick range continuously after removing the center deadzone."""
        if not math.isfinite(value) or abs(value) <= self.deadzone:
            return 0.0
        return math.copysign((min(abs(value), 1.0) - self.deadzone) /
                             (1.0 - self.deadzone), value)

    def joy_callback(self, msg: Joy):
        """Update toggles on rising edges; zero-turn overrides them while held."""
        def button(name):
            index = self.buttons_map.get(name, -1)
            return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

        holding_zero = button('hold_zero_turn')
        for name, mode in (('toggle_ackermann', MODE_ACKERMANN),
                           ('toggle_crab_walk', MODE_CRAB_WALK)):
            pressed = button(name)
            if not holding_zero and pressed and not self.last_button_states.get(name, False):
                self.drive_mode = mode
            # Track edges even during a held override to avoid synthetic presses on release.
            self.last_button_states[name] = pressed

        def axis(name):
            index = self.axes_map.get(name, -1)
            return msg.axes[index] if 0 <= index < len(msg.axes) else 0.0

        mode = MODE_ZERO_TURN if holding_zero else self.drive_mode
        command = project_command(
            mode, self.apply_deadzone(axis('throttle')) * self.linear_scale,
            self.apply_deadzone(axis('steering')) * self.angular_scale)
        self.command_publisher_.publish(command)


def main(args=None):
    """Start joystick processing and release ROS resources on shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
