cmake_minimum_required(VERSION 2.8.3)
project(ros_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_executable(imu_node src/imu_node.cpp)

target_link_libraries(imu_node
  ${catkin_LIBRARIES}
)