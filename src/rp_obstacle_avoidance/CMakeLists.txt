cmake_minimum_required(VERSION 2.8.3)
project(rp_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs rp_locomotion)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs rp_locomotion
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
add_executable(rp_obstacle_avoidance_node src/obstacle_avoidance.cpp)

add_dependencies(rp_obstacle_avoidance_node rp_locomotion)

target_link_libraries(rp_obstacle_avoidance_node
  ${catkin_LIBRARIES}
)
