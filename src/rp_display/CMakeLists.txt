cmake_minimum_required(VERSION 2.8.3)
project(rp_display)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rp_display_node src/display.cpp)

target_link_libraries(rp_display_node
  ${catkin_LIBRARIES}
)
