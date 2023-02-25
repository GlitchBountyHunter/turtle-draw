cmake_minimum_required(VERSION 3.0.2)
project(turtle_sub_cmdvel)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
#  message_generation
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(turtle_go_sub src/turtle_go_sub.cpp)
add_dependencies(turtle_go_sub ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtle_go_sub ${catkin_LIBRARIES})