import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/crazyflie/Documents/Sharfi/mocap_ws/ros2_ws/install/crazyflie_py'
