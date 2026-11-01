"""Verify project launch resolves upstream parameter placeholders."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import yaml


def test_navigation_resolves_parameters():
    path = Path(__file__).resolve().parents[1] / 'launch/navigation.launch.py'
    spec = importlib.util.spec_from_file_location('navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if not isinstance(action, Node):
            action.execute(context)
        else:
            if len(action._Node__parameters) != 2:
                continue
            parameters = evaluate_parameters(context, action._Node__parameters)
            if isinstance(parameters[0], dict):
                continue
            text = Path(parameters[0]).read_text()
            config = yaml.safe_load(text)
            for name in ('local_costmap', 'global_costmap'):
                costmap = config[name][name]['ros__parameters']
                assert costmap['keepout_filter']['enabled'] is False
            assert config['global_costmap']['global_costmap']['ros__parameters']['speed_filter']['enabled'] is False
            assert '$(find-pkg-share' not in text
