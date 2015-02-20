cmake_minimum_required(VERSION 2.8.3)
project(rose_laser_filter)

find_package(catkin 
	REQUIRED
		roscpp
		sensor_msgs
)

catkin_package(
	INCLUDE_DIRS 
		include
	LIBRARIES
	CATKIN_DEPENDS
		roscpp
		sensor_msgs
#  DEPENDS system_lib
)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") # TODO OH: -g is DEBUG FLAG

include_directories(include ${catkin_INCLUDE_DIRS})

# Laser filter node
add_executable(rose_laser_filter 
				src/laser_clear_filter.cpp
				src/laser_clear_filter_node.cpp)	

add_dependencies(rose_laser_filter ${catkin_EXPORTED_TARGETS}_generate_messages_cpp)

target_link_libraries(rose_laser_filter ${catkin_LIBRARIES})
