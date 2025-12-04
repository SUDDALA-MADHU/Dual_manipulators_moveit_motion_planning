cmake_minimum_required(VERSION 3.0.2)
project(cobot_mesh_loader)

## -----------------------------------------------
## Find catkin and system dependencies
## -----------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  gazebo_msgs
  roslib
  gazebo_ros_link_attacher
  trajectory_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

## -----------------------------------------------
## Declare catkin package
## -----------------------------------------------
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
    rospy 
    std_msgs
    gazebo_msgs
    roslib
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
    gazebo_ros_link_attacher
    trajectory_msgs
    geometry_msgs
)


## -----------------------------------------------
## Include directories
## -----------------------------------------------
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## -----------------------------------------------
## Build your pick_place node
## -----------------------------------------------
add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES})
add_executable(spawn_box src/spawn_box.cpp)
target_link_libraries(spawn_box ${catkin_LIBRARIES})


add_executable(pick_place_gazebo src/pick_place_gazebo.cpp)
add_dependencies(pick_place_gazebo
  gazebo_ros_link_attacher_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pick_place_gazebo ${catkin_LIBRARIES})


