cmake_minimum_required(VERSION 3.0.2)
project(teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  tf2
  tf2_ros
  message_generation
)

# catkin_python_setup()

add_service_files(
  FILES
  GetBoxLimits.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/teleop_tf_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
