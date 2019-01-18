cmake_minimum_required(VERSION 2.8.3)
project(sv_clustering)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(PCL 1.9 REQUIRED)


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
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




add_executable(sv_clustering src/main.cpp src/sv_clustering.cpp)
target_link_libraries(sv_clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
