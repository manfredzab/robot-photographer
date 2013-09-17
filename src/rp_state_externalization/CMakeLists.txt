cmake_minimum_required(VERSION 2.8.3)
project(rp_state_externalization)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp rp_autonomous_photography rp_navigation rp_framing rp_locomotion rp_head_tracking)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp rp_autonomous_photography rp_navigation rp_framing rp_locomotion rp_head_tracking
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rp_state_externalization_node src/state_externalization.cpp)

add_dependencies(rp_state_externalization_node rp_autonomous_photography rp_navigation rp_framing rp_locomotion rp_head_tracking)

target_link_libraries(rp_state_externalization_node
  ${catkin_LIBRARIES}
)
