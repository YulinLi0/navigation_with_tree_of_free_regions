cmake_minimum_required(VERSION 3.2.0)
project(pop_planner)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
    laser_geometry
    geometry_msgs
    message_filters
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
    frtree_local_map
    decomp_ros_msgs
    decomp_ros_utils
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

# find_package(jps3d REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


set(CMAKE_PREFIX_PATH "/opt/rexlab/altro")
find_package(altro REQUIRED)
find_library(COPT_LIB PATHS /opt/copt72/lib)
link_directories(/opt/copt72/lib)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES pop_planner 
 CATKIN_DEPENDS geometry_msgs roscpp roscpp rospy sensor_msgs std_msgs visualization_msgs frtree_local_map
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  # ${JPS3D_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../DecompROS/DecompUtil/include
  ${PROJECT_SOURCE_DIR}/../third_party/geo_utils
  /opt/copt72/include/coptcpp_inc
  /opt/copt72/include
  ${PROJECT_SOURCE_DIR}/include/da
)

add_executable(${PROJECT_NAME}_node
    src/dynamic_plan_manager_pop.cpp 
    src/dynamic_plan_fsm_pop.cpp 
    src/global_graph.cpp
    src/Altro/altro_constraints.cpp
    src/Altro/altro_cost.cpp
    src/Altro/altro_model.cpp
    src/Altro/altro_problem.cpp
    src/SDPsolver/SDPsolver.cpp
  )

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  # ${JPS3D_LIBRARIES}
  altro::altro 
  fmt::fmt
  /opt/copt72/lib/libcopt_cpp.so
)
