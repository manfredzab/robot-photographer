cmake_minimum_required(VERSION 2.8.3)
project(rp_navigation)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs roscpp rp_locomotion)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs std_msgs roscpp rp_locomotion
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rp_navigation_node src/navigation.cpp)

add_dependencies(rp_navigation_node rp_locomotion)

target_link_libraries(rp_navigation_node
  ${catkin_LIBRARIES}
)
