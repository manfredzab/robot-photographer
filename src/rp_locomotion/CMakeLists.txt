cmake_minimum_required(VERSION 2.8.3)
project(rp_locomotion)

find_package(catkin REQUIRED COMPONENTS geometry_msgs std_msgs kobuki_msgs roscpp)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS geometry_msgs std_msgs roscpp kobuki_msgs
)

include_directories(include
  ../rp_navigation/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rp_locomotion_node src/locomotion.cpp)

target_link_libraries(rp_locomotion_node
  ${catkin_LIBRARIES}
)
