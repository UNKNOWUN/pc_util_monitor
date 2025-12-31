import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kamiko/ros2_setup_scripts/ros2_ws/src/pc_util_monitor/_ws/install/pc_util_monitor'
