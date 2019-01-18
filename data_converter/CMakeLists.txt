cmake_minimum_required(VERSION 2.8.3)
project(data_converter)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_converter
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
)



add_executable(rosbag_reader src/rosbag_reader.cpp)
target_link_libraries(rosbag_reader ${catkin_LIBRARIES} )
add_executable(data_converter src/data_converter.cpp src/mean_shift.cpp)
target_link_libraries(data_converter ${catkin_LIBRARIES} ${HEADER_FILES})
