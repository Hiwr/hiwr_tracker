cmake_minimum_required(VERSION 2.8.3)
project(hiwr_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs #Point
  sensor_msgs #ROI
  dynamixel_msgs #Joint_state
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs dynamixel_msgs
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++0x -pthread")



add_executable(hiwr_tracker src/hiwr_tracker.cpp)
target_link_libraries(hiwr_tracker ${catkin_LIBRARIES})
add_dependencies(hiwr_tracker ${catkin_EXPORTED_TARGETS})

