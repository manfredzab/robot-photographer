cmake_minimum_required(VERSION 2.8.3)
project(rp_autonomous_photography)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp rp_navigation rp_framing rp_camera rp_uploader rp_locomotion rp_head_tracking)
find_package(
  OpenCV REQUIRED
)


catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp rp_navigation rp_framing rp_camera rp_uploader rp_locomotion rp_head_tracking
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rp_autonomous_photography_node src/autonomous_photography.cpp)

add_dependencies(rp_autonomous_photography_node rp_navigation rp_framing rp_camera rp_uploader rp_locomotion rp_head_tracking)

target_link_libraries(rp_autonomous_photography_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
