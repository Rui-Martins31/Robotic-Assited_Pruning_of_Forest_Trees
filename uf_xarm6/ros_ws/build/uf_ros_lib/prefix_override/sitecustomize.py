import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/castrolab-agx-orin/Documents/Robotic-Assited_Pruning_of_Forest_Trees/uf_xarm6/ros_ws/install/uf_ros_lib'
