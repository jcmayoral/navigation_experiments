cmake_minimum_required(VERSION 2.8.3)
project(fake_imu)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_python_setup()
catkin_package()
