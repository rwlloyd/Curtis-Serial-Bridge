cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
)


include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(v4_drive_controller SHARED v4_drive_controller.cpp)
target_link_libraries(v4_drive_controller ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
