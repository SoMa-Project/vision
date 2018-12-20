cmake_minimum_required(VERSION 2.8.3)
project(ecto_rbo_grasping)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules # for tinyxml
  roscpp
  ecto
  ecto_pcl
  ecto_ros
  tf_conversions
  eigen_conversions
  pregrasp_msgs
  geometry_graph_msgs
  tf
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#find_package(PCL 1.7 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen 3 REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core )

find_package(TinyXML REQUIRED)

if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )
else()
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

set(BUILD_MANIFOLD_OPTIMIZATION "FALSE" CACHE BOOL "Optional library that offers a few cells.")


IF (BUILD_MANIFOLD_OPTIMIZATION)
    find_library(NLOPT_LIBRARY nlopt /usr/local/lib)
    IF (DEFINED NLOPT_LIBRARY-NOTFOUND)
        MESSAGE (FATAL_ERROR "NLOpt missing! Please install: http://ab-initio.mit.edu/wiki/index.php/NLopt \n If you have problems on a 64-bit machine with -fPIC check this out: https://www.mail-archive.com/nlopt-discuss@ab-initio.mit.edu/msg00018.html")
    ENDIF (DEFINED NLOPT_LIBRARY-NOTFOUND)
ENDIF ()

find_library(GDIAM_LIBRARY libgdiam.so /usr/local/lib)
IF (DEFINED GDIAM_LIBRARY-NOTFOUND)
        MESSAGE (FATAL_ERROR "gdiam missing! Please install: http://sarielhp.org/research/papers/00/diameter/")
ENDIF (DEFINED GDIAM_LIBRARY-NOTFOUND)

IF (NOT DEFINED ENV{WM5_PATH})
    MESSAGE (FATAL_ERROR "Wild Magic Engine 5.14 missing! Please install: https://www.geometrictools.com/Downloads/WildMagic5p14.zip (from https://www.geometrictools.com/Downloads/Downloads.html) \n Installation instructions: https://www.geometrictools.com/Downloads/Wm5p14InstallationRelease.pdf")
endif()



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES ecto_rbo_grasping
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
    $ENV{WM5_PATH}/Include/
)

IF (BUILD_MANIFOLD_OPTIMIZATION)
    add_library(manifold_optimizations
        src/ecto_rbo_grasping/SphericalManifold.cpp
        src/ecto_rbo_grasping/CylindricalManifold.cpp
    #    src/ecto_rbo_grasping/EnvironmentalConstraintRegion.cpp
        include/ecto_rbo_grasping/SphericalManifold.h  # for QtCreator
        include/ecto_rbo_grasping/CylindricalManifold.h
    #    include/ecto_rbo_grasping/EnvironmentalConstraintRegion.h
    )
    
    target_link_libraries(manifold_optimizations
        ${NLOPT_LIBRARY}
    )
ENDIF ()

add_library(pose_sets
#    include/ecto_rbo_grasping/PoseSet.h
#    include/ecto_rbo_grasping/OrientationSet.h
    src/ecto_rbo_grasping/PoseSet.cpp
    src/ecto_rbo_grasping/OrientationSet.cpp
#    src/ecto_rbo_grasping/gdiam.cpp
)

target_link_libraries(pose_sets
    ${catkin_LIBRARIES}
    ${CGAL_LIBRARIES}
    ${CGAL_3RD_PARTY_LIBRARIES}
    ${GDIAM_LIBRARY}
    ${TinyXML_LIBRARIES}
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Imagics.so
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Physics.so
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Core.so
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Applications.so
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Mathematics.so
    $ENV{WM5_PATH}/Library/ReleaseDynamic/libWm5Graphics.so
)

# generate ecto module
set(ECTO_CELL_SRCS
    src/ecto_rbo_grasping/module.cpp
#    src/ecto_rbo_grasping/BroadcastBoxGrasps.cpp
#    src/ecto_rbo_grasping/BroadcastDiskGrasps.cpp
#    src/ecto_rbo_grasping/BroadcastGrasps.cpp
    src/ecto_rbo_grasping/CreateGrasps.cpp
    src/ecto_rbo_grasping/IfcoGrasps.cpp
    src/ecto_rbo_grasping/PublishGrasps.cpp
    src/ecto_rbo_grasping/MergeGraspStrategies.cpp
#    src/ecto_rbo_grasping/MergeManifolds.cpp
#    src/ecto_rbo_grasping/FilterPreGrasps.cpp
#    src/ecto_rbo_grasping/EdgeExploitingGrasps.cpp
#    src/ecto_rbo_grasping/PointFeatureHistogram.cpp
#    src/ecto_rbo_grasping/GraspStrategyArraySubscriber.cpp
#    src/ecto_rbo_grasping/ApproachBoxGrasp.cpp
    src/ecto_rbo_grasping/CliffGrasps.cpp
    src/ecto_rbo_grasping/WallGrasps.cpp
#    src/ecto_rbo_grasping/WallGraspsFromModel.cpp
#    src/ecto_rbo_grasping/PushingMotions.cpp
#    src/ecto_rbo_grasping/PushingSeeds.cpp
#    src/ecto_rbo_grasping/SlidingSeeds.cpp
#    src/ecto_rbo_grasping/LandingMotions.cpp
    src/ecto_rbo_grasping/PositioningMotions.cpp
#    src/ecto_rbo_grasping/PlanGrasps.cpp
#    src/ecto_rbo_grasping/ManifoldsAtEdges.cpp
    src/ecto_rbo_grasping/ManifoldsPlanar.cpp
#    src/ecto_rbo_grasping/WallSlides.cpp
#    src/ecto_rbo_grasping/PublishManifolds.cpp
#    src/ecto_rbo_grasping/FilterManifolds.cpp
#    src/ecto_rbo_grasping/PublishGraspStrategySequence.cpp
    src/ecto_rbo_grasping/PublishObjectList.cpp
    src/ecto_rbo_grasping/CreateGeometryGraph.cpp
    src/ecto_rbo_grasping/PublishGeometryGraph.cpp
#    src/ecto_rbo_grasping/TransitionSlideEdgeGrasp.cpp
#    src/ecto_rbo_grasping/TransitionPressSlide.cpp
#    src/ecto_rbo_grasping/OrientationTest.cpp
)

set(ECTO_CELL_LINKED_LIBS
    pose_sets
    ${PCL_LIBRARIES}
    ${ecto_PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${TinyXML_LIBRARIES}
)

IF (BUILD_MANIFOLD_OPTIMIZATION)
    list(APPEND ECTO_CELL_SRCS
        src/ecto_rbo_grasping/BroadcastSphereGrasps.cpp
        src/ecto_rbo_grasping/BroadcastCylinderGrasps.cpp
    )
    list(APPEND ECTO_CELL_LINKED_LIBS
        manifold_optimizations
    )
ENDIF ()

ectomodule(ecto_rbo_grasping INSTALL
    ${ECTO_CELL_SRCS}
)

link_ecto(ecto_rbo_grasping
    ${ECTO_CELL_LINKED_LIBS}
)


## Declare a cpp executable
#add_executable(ecto_rbo_grasping_node src/ecto_rbo_grasping/gdiam_test.cpp)
#target_link_libraries(ecto_rbo_grasping_node
#    pose_sets
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ecto_rbo_grasping_node ecto_rbo_grasping_generate_messages_cpp)
add_dependencies(pose_sets pregrasp_msgs_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ecto_rbo_grasping_ectomodule
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ecto_rbo_grasping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
