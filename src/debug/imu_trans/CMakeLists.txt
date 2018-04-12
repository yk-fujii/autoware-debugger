cmake_minimum_required(VERSION 2.8.3)
project(imu_trans)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_msgs
  sensor_msgs
  tf2
)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES libnode_template
  CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs sensor_msgs tf2
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(imu_trans
  nodes/imu_trans.cpp)
  add_dependencies(imu_trans ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_trans ${catkin_LIBRARIES})
