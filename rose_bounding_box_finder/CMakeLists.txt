cmake_minimum_required(VERSION 2.8.3)
project(rose_bounding_box_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    image_geometry
    message_generation
    message_runtime
    pcl_conversions
    pcl_ros
    roscomm
    roscpp
    rose_common
    rose_conversions  
    rose_geometry
    rose_transformations
    sensor_msgs
    server_multiple_client
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

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

#Settings for Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/bounding_box_finder.cfg
 )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   BoundingBox.msg
   BoundingBoxVector.msg
   uv_bounding_box.msg
   uv_point.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  convert_bb_to_uv.srv
  toggle.srv
)

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   getboundingbox.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages( 
  DEPENDENCIES 
 	  actionlib_msgs 
 	  roscomm
    rose_bounding_box_finder
 )


##############TEMPORARY##########
find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


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
  INCLUDE_DIRS 
    include
  LIBRARIES 
    bounding_box_finder
  CATKIN_DEPENDS 
    dynamic_reconfigure
    image_geometry
    message_generation
    message_runtime
    pcl_conversions
    pcl_ros
    roscomm
    roscpp
    rose_common
    rose_conversions  
    rose_geometry
    rose_transformations
    sensor_msgs
    server_multiple_client
  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(bounding_box_finder
#   src/${PROJECT_NAME}/bounding_box_finder.cpp
# )

## Declare a cpp executable
add_executable(rose_bounding_box_finder src/bounding_box_finder_node.cpp
								                    src/bounding_box_finder.cpp
)

add_executable(bounding_box_convert_srv src/bounding_box_convert_srv_node.cpp
                                        src/bounding_box_convert_srv.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies( rose_bounding_box_finder ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp
${PROJECT_NAME}_gencfg)

add_dependencies( bounding_box_convert_srv ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp
${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(rose_bounding_box_finder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(bounding_box_convert_srv
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
