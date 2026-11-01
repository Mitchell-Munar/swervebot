"""Guard against parent Nav2 launch arguments replacing mux configuration."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml


def test_mux_uses_own_configuration():
    path = Path(__file__).resolve().parents[1] / 'launch/mux.launch.py'
    spec = importlib.util.spec_from_file_location('mux_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['params_file'] = '/unrelated/nav2.yaml'
    for action in module.generate_launch_description().entities:
        if isinstance(action, Node):
            parameters = evaluate_parameters(context, action._Node__parameters)
            assert parameters[-1]['use_stamped'] is False
            config = Path(parameters[0])
            assert config.name == 'twist_mux.yaml'
            topics = yaml.safe_load(config.read_text())['twist_mux']['ros__parameters']['topics']
            assert topics['teleop']['topic'] == 'cmd_vel_teleop'
            assert topics['teleop']['priority'] > topics['navigation']['priority']
        else:
            action.execute(context)
