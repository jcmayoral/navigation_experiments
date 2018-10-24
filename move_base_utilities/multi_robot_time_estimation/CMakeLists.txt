cmake_minimum_required(VERSION 2.8.3)
project(multi_robots_comm)

find_package(catkin REQUIRED COMPONENTS
             genmsg
             rospy
             std_msgs
             message_generation
             message_runtime)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ContractNetMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs
)

install(PROGRAMS nodes/client.py nodes/server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
