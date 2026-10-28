"""Verify manual mode projection and button transitions independently of hardware."""

from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Joy
from swervebot_teleop.teleop_node import MODE_ACKERMANN, MODE_CRAB_WALK, MODE_ZERO_TURN
from swervebot_teleop.teleop_node import project_command, TeleopNode


@pytest.mark.parametrize('mode,throttle,steering,expected', [
    (MODE_ACKERMANN, 0.2, 0.3, (0.2, 0.0, 0.3)),
    (MODE_ACKERMANN, -0.2, 0.3, (-0.2, 0.0, 0.3)),
    (MODE_ACKERMANN, 0.0, 0.3, (0.0, 0.0, 0.0)),
    (MODE_CRAB_WALK, 0.2, 0.3, (0.2, 0.3, 0.0)),
    (MODE_ZERO_TURN, 0.2, 0.3, (0.0, 0.0, 0.3)),
    (255, 0.2, 0.3, (0.0, 0.0, 0.0)),
    (MODE_CRAB_WALK, float('nan'), 0.3, (0.0, 0.0, 0.0)),
])
def test_projection(mode, throttle, steering, expected):
    """Project each mode into a standard planar Twist, including stopped Ackermann yaw."""
    command = project_command(mode, throttle, steering)
    assert (command.linear.x, command.linear.y, command.angular.z) == expected


def test_buttons_and_held_override():
    """Keep the toggled mode when zero-turn is released and tolerate missing inputs."""
    messages = []
    node = SimpleNamespace(
        drive_mode=MODE_ACKERMANN, last_button_states={},
        axes_map={'throttle': 0, 'steering': 1},
        buttons_map={'toggle_ackermann': 0, 'toggle_crab_walk': 1, 'hold_zero_turn': 2},
        linear_scale=1.0, angular_scale=1.0, apply_deadzone=lambda value: value,
        command_publisher_=SimpleNamespace(publish=messages.append))
    for buttons in ([0, 1, 0], [0, 1, 1], [0, 1, 0]):
        TeleopNode.joy_callback(node, Joy(axes=[0.2, 0.3], buttons=buttons))
    assert messages[0].linear.y == pytest.approx(0.3)
    assert messages[1].linear.x == 0.0
    assert messages[1].angular.z == pytest.approx(0.3)
    assert messages[2].linear.y == pytest.approx(0.3)
    TeleopNode.joy_callback(node, Joy())
    assert messages[-1].linear.x == messages[-1].linear.y == messages[-1].angular.z == 0.0


def test_deadzone_and_invalid_profile(tmp_path):
    """Deadzone scaling stays finite and malformed profiles fail explicitly."""
    node = SimpleNamespace(deadzone=0.1)
    assert TeleopNode.apply_deadzone(node, 0.05) == 0.0
    assert TeleopNode.apply_deadzone(node, 0.55) == pytest.approx(0.5)
    assert TeleopNode.apply_deadzone(node, -2.0) == -1.0
    assert TeleopNode.apply_deadzone(node, float('nan')) == 0.0
    profile = tmp_path / 'bad.yaml'
    profile.write_text('settings: {deadzone: 1.0}')
    node.get_logger = lambda: SimpleNamespace(error=lambda message: None)
    with pytest.raises(ValueError, match='Invalid joystick profile'):
        TeleopNode.load_config(node, profile)
