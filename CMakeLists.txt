cmake_minimum_required(VERSION 2.8.3)
project(walkman_ros_trajgen)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        message_generation
        rosmvnbiomech
        mvnbiomech_ros
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
add_message_files(
  FILES
  JointLinearInterpolationParameters.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  JointLinearInterpolation.srv
  WBJointLinearInterpolation.srv
  SetPose.srv
  PreRecordedTrajectory.srv
  JointLinearTrajectory.srv
  PoseInterpolation.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  walkman_ros_trajgen
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
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
#  LIBRARIES mvnbiomech
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
        include
        include/mvnbiomech
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

file(GLOB MVNSOURCES src/mvnbiomech/*.cpp)
file(GLOB MVNHEADERS include/mvnbiomech/*.h)

## Declare a C++ library
add_library(mvnbiomech SHARED
        ${MVNHEADERS}
        ${MVNSOURCES}
        )



file(GLOB WALKMAN_TRAJGEN_SOURCES src/walkman_trajgen/*.cpp)
file(GLOB WALKMAN_TRAJGEN_HEADERS include/walkman_trajgen/*.h)
   
# C++ library
add_library(walkman_trajgen SHARED
        ${WALKMAN_TRAJGEN_HEADERS}
        ${WALKMAN_TRAJGEN_SOURCES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(walkman_ros_trajgen ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## C++ executable
add_executable(joint_trajgen src/joint-trajgen-pub.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joint_trajgen
        ${catkin_LIBRARIES}
        )

## Cmake target dependencies
add_dependencies(joint_trajgen
        ${PROJECT_NAME}_gencpp
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

### C++ executable
#add_executable(joint_trajgen_mvnbiomech src/joint-trajgen-and-mvnbiomech.cpp)
#
### Specify libraries to link a library or executable target against
#target_link_libraries(joint_trajgen_mvnbiomech
#        ${catkin_LIBRARIES}
#        mvnbiomech
#        )
#
### Cmake target dependencies
#add_dependencies(joint_trajgen_mvnbiomech
#        ${PROJECT_NAME}_gencpp
#        ${${PROJECT_NAME}_EXPORTED_TARGETS}
#        ${catkin_EXPORTED_TARGETS}
#        )

## C++ executable
add_executable(mvnbiomech_jointstate_logger src/mvnbiomech-jointstate-logger.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mvnbiomech_jointstate_logger
        ${catkin_LIBRARIES}
        mvnbiomech
        )

## Cmake target dependencies
add_dependencies(mvnbiomech_jointstate_logger
        ${PROJECT_NAME}_gencpp
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
