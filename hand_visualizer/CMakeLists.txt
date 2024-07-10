cmake_minimum_required(VERSION 3.0.2)
project(hand_visualizer)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  io_msgs
  urdf
  xacro
  tf2_ros
  tf2
  cv_bridge
  topic_tools
  )
find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES foot_visualizer
 CATKIN_DEPENDS roscpp rospy std_msgs urdf xacro tf2_ros cv_bridge
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(hand_visualizer src/main.cpp src/hand_visualizer.h src/hand_visualizer.cpp)
target_link_libraries(hand_visualizer 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)
