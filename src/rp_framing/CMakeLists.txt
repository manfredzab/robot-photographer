cmake_minimum_required(VERSION 2.8.3)
project(rp_framing)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs cv_bridge rp_head_tracking rp_locomotion message_generation)
find_package(
  OpenCV REQUIRED
)

add_message_files(
  FILES
  Frame.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs rp_head_tracking rp_locomotion message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rp_framing_node 
  src/framing.cpp
  ../rp_head_tracking/src/depth_image_processor.cpp
)

add_dependencies(rp_framing_node rp_head_tracking rp_locomotion rp_framing_generate_messages_cpp)

target_link_libraries(rp_framing_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)