cmake_minimum_required(VERSION 2.8.3)
project(rp_camera)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)
find_package(
  Gphoto2 REQUIRED
)

add_service_files(
   FILES
   PhotoService.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rp_camera_node src/camera.cpp src/gphoto2_handler.cpp src/usb_handler.cpp)

add_dependencies(rp_camera_node rp_camera_generate_messages_cpp)

target_link_libraries(rp_camera_node
  ${catkin_LIBRARIES}
  ${Gphoto2_LIBRARIES}
)