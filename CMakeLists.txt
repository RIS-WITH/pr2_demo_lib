cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
project(pr2_demo_lib)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  pr2_controllers_msgs
  actionlib
  sound_play
  tf2_geometry_msgs
  tf2_ros
  roslib
  navigation_position_refinement
)

# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pr2_demo_lib

  # CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  rosbridgecpp
)
add_subdirectory(rosbridgecpp rosbridgecpp)

add_library(${PROJECT_NAME}
  src/Actions.cpp
  src/CsvConfig.cpp
  src/Pr2Robot.cpp
  src/Task.cpp
  rosbridgecpp/rosbridge_ws_client.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} rosbridgecpp)