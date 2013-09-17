cmake_minimum_required(VERSION 2.8.3)
project(rp_head_tracking)

find_package(
  catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs image_transport cv_bridge message_generation
)
find_package(
  OpenCV REQUIRED
)

add_message_files(
  FILES
  Heads.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rp_head_tracking_node 
  src/head_tracking.cpp 
  src/heads_message_builder.cpp 
  src/kernel_logistic_regression_classifier.cpp 
  src/depth_image_processor.cpp 
  src/color_image_processor.cpp 
  src/depth_head_detector.cpp
  src/color_face_detector.cpp
  src/depth_head_tracker.cpp
  src/kernel_logistic_regression_classifier_kernel.cpp
  src/bayesian_skin_classifier.cpp
)

add_executable(rp_mock_head_tracking_node 
  src/mock_head_tracking.cpp 
  src/heads_message_builder.cpp
)

add_dependencies(rp_head_tracking_node rp_head_tracking_generate_messages_cpp)
add_dependencies(rp_mock_head_tracking_node rp_head_tracking_generate_messages_cpp)

target_link_libraries(
  rp_head_tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(
  rp_mock_head_tracking_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)