cmake_minimum_required(VERSION 3.0.2)
project(octomap_waypoint_plugin)

find_package(catkin REQUIRED COMPONENTS uav_ros_tracker uav_ros_msgs
                                        larics_motion_planning)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES octomap_waypoint_plugin CATKIN_DEPENDS
  # other_catkin_pkg DEPENDS system_lib
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(octomap_planner_client src/octomap_planner_client.cpp)
add_dependencies(octomap_planner_client ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(octomap_planner_client ${catkin_LIBRARIES})

# Mark executables and/or libraries for installation
install(
  TARGETS octomap_planner_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
