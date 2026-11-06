"""Check selection precedence and SDL enumeration parsing without hardware."""

import importlib.util
from pathlib import Path

import pytest

_spec = importlib.util.spec_from_file_location(
    'teleop_launch', Path(__file__).parents[1] / 'launch/teleop.launch.py')
launch_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(launch_module)


@pytest.mark.parametrize('device,override,name,expected', [
    ({}, '', '', (0, '')),
    ({'id': 1}, '', '', (1, '')),
    ({'name': 'Xbox'}, '', '', (0, 'Xbox')),
    ({'name': 'Xbox'}, '2', '', (2, '')),
    ({'id': 1}, '2', 'Other', (2, 'Other')),
])
def test_selection(device, override, name, expected):
    """Explicit launch choices override profile choices."""
    assert launch_module.resolve_device(device, override, name) == expected


@pytest.mark.parametrize('device', [{'id': -1}, {'id': True}, {'name': 2}])
def test_invalid_selection(device):
    """Reject malformed device fields before starting the driver."""
    with pytest.raises(ValueError):
        launch_module.resolve_device(device)


def test_enumeration_rows():
    """Count device rows without counting table headings."""
    assert len(launch_module.device_rows('ID : Name\n---\n 0 : Keyboard\n 1 : Xbox')) == 2
