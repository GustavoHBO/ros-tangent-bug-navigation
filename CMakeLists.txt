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
## Enable testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  ## Option 1: Using catkin_add_gtest macro
  catkin_add_gtest(test_tangentbug test/test_tangentbug.cpp src/TangentBug.cpp)
  if(TARGET test_tangentbug)
    target_link_libraries(test_tangentbug ${catkin_LIBRARIES})
  endif()

  ## Option 2: Directly finding GTest (if needed)
  # find_package(GTest REQUIRED)
  # include_directories(${GTEST_INCLUDE_DIRS})
  # add_executable(test_tangentbug test/test_tangentbug.cpp src/TangentBug.cpp)
  # target_link_libraries(test_tangentbug ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  # add_rostest_gtest(test_tangentbug test/test_tangentbug.launch test/test_tangentbug.cpp)
endif()
