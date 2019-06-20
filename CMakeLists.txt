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

add_executable(map_transformer src/map_transformer.cpp)
target_link_libraries(map_transformer
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)

add_executable(map_filter src/map_filter.cpp)
target_link_libraries(map_filter
${catkin_LIBRARIES}
${libpointmatcher_LIBRARIES}
)

add_executable(loc_demo src/loc_demo.cpp)
target_link_libraries(loc_demo
${catkin_LIBRARIES}
)

add_executable(laser_reg src/laser_reg.cpp)
target_link_libraries(laser_reg
${catkin_LIBRARIES}
)

add_executable(odom_may src/odom_may.cpp)
target_link_libraries(odom_may
${catkin_LIBRARIES}
)

add_executable(sender_udp src/sender_udp.cpp)
target_link_libraries(sender_udp
${catkin_LIBRARIES}
)

add_executable(recorder src/recorder.cpp)
target_link_libraries(recorder
${catkin_LIBRARIES}
)



