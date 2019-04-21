cmake_minimum_required(VERSION 2.8.3)
project(pa2)

find_package(catkin REQUIRED
  rospy
  message_generation
  std_msgs
)


add_message_files(
  FILES
  Wifi.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_install_python(PROGRAMS
   scripts/wifi_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
   scripts/robot_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
