cmake_minimum_required(VERSION 2.8.3)
project(rp_speech)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp)
find_library(
  espeak REQUIRED
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS std_msgs roscpp espeak
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${espeak_INCLUDE_DIRS}
)

add_executable(rp_speech_node src/speech.cpp)

target_link_libraries(rp_speech_node
  ${catkin_LIBRARIES}
  espeak
)
