cmake_minimum_required(VERSION 2.8.3)
project(ecto_rbo_pcl)

# to remove warnings
cmake_policy(SET CMP0022 OLD)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ecto
  ecto_pcl
  ecto_ros
  sensor_msgs
  tf
  tf_conversions
  eigen_conversions
  rosbag
  roslib
  pcl_msgs
  pcl_conversions
  ifco_pose_estimator
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

## System dependencies are found with CMake's conventions
find_package(PythonLibs REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(OpenCV REQUIRED)

# IMPORTANT: Needs to come AFTER find_package(PCL ...) since it is overwritten by it
find_package(Boost REQUIRED COMPONENTS python)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  #we must not set the libraries flag because ecto reove the lib prefix from the .so files
  #and referencing catkin packages will throw a build error because they cannot find it
	LIBRARIES ecto_rbo_pcl_python
  CATKIN_DEPENDS ecto ecto_pcl ecto_ros
  DEPENDS system_lib
)

###########
## Build ##
###########

set( CMAKE_C_FLAGS   "-ftime-report -fno-stack-protector" )
set( CMAKE_CXX_FLAGS "-ftime-report -fno-stack-protector" )


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${EIGEN_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

add_library (ecto_rbo_pcl_python src/ecto_rbo_pcl/boost_python_converters.cpp)
target_link_libraries(ecto_rbo_pcl_python  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
set_target_properties(ecto_rbo_pcl_python
                      PROPERTIES PREFIX "")

## Declare a cpp library
#add_library(${PROJECT_NAME}_utils
#		# for QtCreator
#		include/${PROJECT_NAME}/HSVHistogramCalculator.h
#		include/${PROJECT_NAME}/region_growing.h
#		include/${PROJECT_NAME}/region_growing_histogram.h
#		include/${PROJECT_NAME}/region_growing.hpp
#		include/${PROJECT_NAME}/region_growing_histogram.hpp
#		include/${PROJECT_NAME}/single_region_growing.h
#		include/${PROJECT_NAME}/single_region_growing.hpp
#		src/${PROJECT_NAME}/HSVHistogramCalculator.cpp
#)
#target_link_libraries(${PROJECT_NAME}_utils
#	${catkin_LIBRARIES}
#	${PCL_LIBRARIES}
#	${OpenCV_LIBRARIES}
#)


# generate templated SpinImageEstimation classes with different bin sizes
#add_library (spin_image_definitions
# src/ecto_rbo_pcl/spin_image_definitions.cpp
# include/ecto_rbo_pcl/spin_image_definitions.h # for QtCreator
# )
#target_link_libraries(spin_image_definitions  ${PCL_LIBRARIES})

# generate ecto module
ectomodule(ecto_rbo_pcl INSTALL
    src/ecto_rbo_pcl/module.cpp
    src/ecto_rbo_pcl/ExtractClosestCluster.cpp
    src/ecto_rbo_pcl/FloodFill.cpp
    src/ecto_rbo_pcl/NormalEstimationIntegral.cpp
    src/ecto_rbo_pcl/CropBox.cpp
    src/ecto_rbo_pcl/Message2PointCloud.cpp
    src/ecto_rbo_pcl/Centroid.cpp
    src/ecto_rbo_pcl/PassThroughOrganized.cpp
    src/ecto_rbo_pcl/PlaneFits.cpp
    src/ecto_rbo_pcl/MergeClusters.cpp
    src/ecto_rbo_pcl/BroadcastTF.cpp
    src/ecto_rbo_pcl/TF2Vector.cpp
    src/ecto_rbo_pcl/Vector2TF.cpp
    src/ecto_rbo_pcl/RemoveNaNFromPointCloud.cpp
    src/ecto_rbo_pcl/RegionGrowingRGB.cpp
    src/ecto_rbo_pcl/FilterClustersBySize.cpp
    src/ecto_rbo_pcl/WrapInVector.cpp
    src/ecto_rbo_pcl/CreateTF.cpp
    src/ecto_rbo_pcl/PCDReader.cpp
    src/ecto_rbo_pcl/IfcoDetection.cpp
    src/ecto_rbo_pcl/BroadcastIfcoSpecifications.cpp
)

link_ecto(ecto_rbo_pcl
#		spin_image_definitions
    ${ecto_PCL_LIBRARIES}
    ${PCL_LIBRARIES}
		${Boost_LIBRARIES}
    ${OpenCV_LIBS}
		${catkin_LIBRARIES}
#		ecto_rbo_pcl_utils
)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#	samples/show_normals.py
#	samples/crop_pcd.py
#	 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS ecto_rbo_pcl_ectomodule
#	 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# copy the boost-python converters defined in ecto_rbo_pcl_python to the
# python dist-packages folder
add_custom_command(TARGET ecto_rbo_pcl_python POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:ecto_rbo_pcl_python> ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/../$<TARGET_FILE_NAME:ecto_rbo_pcl_python>
)

#catkin_install_python(PROGRAMS scripts/your_node1 scripts/your_node2
#                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()


## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ecto_rbo_pcl.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
