import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samsanchez/Documents/Robotics/ROS/ROS2-Humble-Turtlesim/ros2_ws/install/my_turtle_controller'
