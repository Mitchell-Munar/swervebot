import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mitchell_munar/Code/ROS2/swervebot/install/web_bridge_demo'
