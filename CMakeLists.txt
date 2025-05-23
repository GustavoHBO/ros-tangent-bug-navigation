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

## Enable testing
enable_testing()

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tangentbug_utils
  src/utils/tangentbug_utils.cpp
)

target_link_libraries(tangentbug_utils
  ${catkin_LIBRARIES}
)

add_executable(tangent_bug_2dnav src/tangent_bug_2dnav.cpp src/TangentBug.cpp src/utils/tangentbug_utils.cpp)
target_link_libraries(tangent_bug_2dnav
  tangentbug_utils
  ${catkin_LIBRARIES}
  ${roslib_LIBRARIES}
)
## Enable testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_tangentbug test/test_tangentbug.cpp src/TangentBug.cpp)
  if(TARGET test_tangentbug)
    target_link_libraries(test_tangentbug
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_detect_obstacle_boundaries test/test_detect_obstacle_boundaries.cpp src/TangentBug.cpp)
  if(TARGET test_detect_obstacle_boundaries)
    target_link_libraries(test_detect_obstacle_boundaries
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_extract_obstacle_segments test/test_extract_obstacle_segments.cpp src/TangentBug.cpp)
  if(TARGET test_extract_obstacle_segments)
    target_link_libraries(test_extract_obstacle_segments
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_find_closest_endpoint
    test/test_find_closest_endpoint.cpp
    src/TangentBug.cpp
  )
  if(TARGET test_find_closest_endpoint)
    target_link_libraries(test_find_closest_endpoint
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_find_best_segment_toward_goal
    test/test_find_best_segment_toward_goal.cpp
    src/TangentBug.cpp
  )
  if(TARGET test_find_best_segment_toward_goal)
    target_link_libraries(test_find_best_segment_toward_goal
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_offset_segment_toward_robot
    test/test_offset_segment_toward_robot.cpp
    src/TangentBug.cpp
  )
  if(TARGET test_offset_segment_toward_robot)
    target_link_libraries(test_offset_segment_toward_robot
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_offset_segment_toward_robot_perspective
    test/test_offset_segment_toward_robot_perspective.cpp
    src/TangentBug.cpp
  )
  if(TARGET test_offset_segment_toward_robot_perspective)
    target_link_libraries(test_offset_segment_toward_robot_perspective
      tangentbug_utils
      ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_simplify_segment test/test_simplify_segment.cpp)
  target_link_libraries(test_simplify_segment
    tangentbug_utils

    ${catkin_LIBRARIES}
  )
endif()
