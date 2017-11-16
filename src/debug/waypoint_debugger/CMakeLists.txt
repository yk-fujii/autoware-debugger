cmake_minimum_required(VERSION 2.8.3)
project(waypoint_debugger)

find_package(catkin REQUIRED COMPONENTS
  gnss
  roscpp
  std_msgs
  tf
  waypoint_follower
  vector_map
  lane_planner
  autoware_msgs
  amathutils
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_follower
  CATKIN_DEPENDS roscpp std_msgs tf waypoint_follower vector_map lane_planner autoware_msgs amathutils
  DEPENDS gnss
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_debugger  nodes/waypoint_debugger/waypoint_debugger.cpp)
target_link_libraries(waypoint_debugger ${catkin_LIBRARIES})
add_dependencies(waypoint_debugger ${catkin_EXPORTED_TARGETS})
