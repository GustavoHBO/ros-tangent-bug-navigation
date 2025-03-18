cmake_minimum_required(VERSION 3.0.2)
project(tangent_bug_2dnav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  roslib
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tangent_bug_2dnav src/tangent_bug_2dnav.cpp src/TangentBug.cpp)
target_link_libraries(tangent_bug_2dnav 
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
)

