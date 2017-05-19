cmake_minimum_required(VERSION 2.8.3)
project(ecto_cerrt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  geometry_msgs
  actionlib
  pcl_conversions
  pcl_msgs
  pcl_ros
  sensor_msgs
  tf
  tf_conversions
  hybrid_automaton_msgs
	moveit_msgs
	moveit_ros_planning 
	moveit_ros_planning_interface
	interactive_markers 
	visualization_msgs
	moveit_core
)

catkin_package(
  #CATKIN_DEPENDS actionlib_msgs  trik_controller moveit_ros_planning_interface 
  #LIBRARIES trik_controller sdformat moveit_ros_planning_interface 
)

add_executable(getPlanes src/getPlanes.cpp)
target_link_libraries(getPlanes ${catkin_LIBRARIES} ${ROS_LIBRARIES})  

