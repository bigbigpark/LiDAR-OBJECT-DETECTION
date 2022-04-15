cmake_minimum_required(VERSION 3.0.2)
project(lidar_object_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  visualization_msgs
  lidar_msgs
)
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES bisa_lane_detection
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp std_msgs visualization_msgs lidar_msgs
  # DEPENDS system_lib Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection src/main.cpp src/object_detection.cpp)
add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_detection
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)