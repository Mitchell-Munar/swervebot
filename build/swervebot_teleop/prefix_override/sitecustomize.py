import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mmunar/Code/ROS2/swerve_bot/install/swervebot_teleop'
