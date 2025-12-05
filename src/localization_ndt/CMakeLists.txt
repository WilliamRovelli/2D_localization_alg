cmake_minimum_required(VERSION 3.0.2)
project(localization_ndt)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  ndt_omp
  tf2_ros          
  tf2
)

find_package(PCL REQUIRED)

find_package(yaml-cpp REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    roslib
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    laser_geometry
    tf2_ros        
    tf2            
    ndt_omp
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(ndt_localizer_node
  src/ndt_localizer_node.cpp
  src/initial_pose_manager.cpp
)

target_link_libraries(ndt_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)
