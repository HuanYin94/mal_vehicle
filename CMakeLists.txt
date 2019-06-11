cmake_minimum_required(VERSION 2.8.3)
project(mal_vehicle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  tf
  tf_conversions
  libpointmatcher_ros
  roslib
)

add_definitions(-std=c++11 -O2 -g -DNDEBUG)

catkin_package (
#	CATKIN_DEPENDS roscpp
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${libpointmatcher_INCLUDE_DIRS}
)

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)

#add_executable(odom_pub src/mapping.cpp)
#target_link_libraries(mapping
#${catkin_LIBRARIES}
#)


