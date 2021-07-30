cmake_minimum_required(VERSION 2.8)
project(object_detection)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	pcl_ros
	tf_conversions
	pcl_conversions
	eigen_conversions
	laser_segmentation
	nodelet
	multi_hypothesis_tracking_msgs
	geometry_msgs
	visualization_msgs
)

catkin_package(
		INCLUDE_DIRS include
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_library(detector
		src/detector.cpp
)

add_dependencies(detector
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
)

target_link_libraries(detector
		${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(object_detection_node
        src/object_detection_node.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(object_detection_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(object_detection_node
		detector
        ${catkin_LIBRARIES}
)

install(TARGETS object_detection_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a C++ library
add_library(object_detection_nodelet
        src/object_detection_nodelet.cpp
)

add_dependencies(object_detection_nodelet
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_detection_nodelet
		detector
        ${catkin_LIBRARIES}
)

install(TARGETS object_detection_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
