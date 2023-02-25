cmake_minimum_required(VERSION 3.0.2)
project(turtle_go)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  message_generation
)
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_library(turtle_go_draw src/turtle_go_draw.cpp src/PID.cpp)

add_executable(turtle_go src/turtle_go.cpp)

target_link_libraries(turtle_go turtle_go_draw ${catkin_LIBRARIES})