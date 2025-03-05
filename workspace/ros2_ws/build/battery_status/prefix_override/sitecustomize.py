import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sagar/Developer/Robotics/Getting-Started-with-ROS2-A-Tutorial-Series/workspace/ros2_ws/install/battery_status'
