cmake_minimum_required(VERSION 3.0.2)
project(frtree_local_map)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  decomp_ros_msgs
  decomp_ros_utils
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES frtree_local_map
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs 
)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/../DecompROS/DecompUtil/include
  ${PROJECT_SOURCE_DIR}/../third_party/geo_utils
)

link_directories(${PCL_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library(${PROJECT_NAME}
  src/local_map.cpp 
  src/raycast.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/local_map_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(local_decomp_node src/local_decomp_node.cpp src/local_decomp.cpp)
target_link_libraries(local_decomp_node ${catkin_LIBRARIES} ${PROJECT_NAME})
