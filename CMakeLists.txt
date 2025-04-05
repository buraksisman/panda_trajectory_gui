cmake_minimum_required(VERSION 3.0.2)
project(panda_trajectory_gui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  trajectory_msgs
  franka_core_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS scripts/panda_trajectory_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
