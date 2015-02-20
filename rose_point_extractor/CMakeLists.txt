cmake_minimum_required(VERSION 2.8.3)
project(rose_point_extractor)

set(CMAKE_CXX_FLAGS "-std=c++0x") 

find_package(catkin REQUIRED
	COMPONENTS
		genmsg
		image_geometry
		pcl_conversions
	    pcl_ros
	    rose_common
		roscpp 
)

add_service_files( DIRECTORY srv 
	FILES
	get_point.srv
)

generate_messages( 
	DEPENDENCIES
	geometry_msgs
)

catkin_package(
   	INCLUDE_DIRS 	
   		include
	CATKIN_DEPENDS  
		rose_common
	DEPENDS 		
		message_generation
)

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose_point_extractor 	src/point_extractor_node.cpp
									src/point_extractor.cpp
)
add_dependencies( rose_point_extractor ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries( rose_point_extractor ${catkin_LIBRARIES} )
