cmake_minimum_required(VERSION 3.0.2)
project(foot_visualizer)


add_compile_options(-std=c++17)
# add_definitions(EXPORT_COMPILE_COMMANDS on)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  io_msgs
  urdf
  xacro
  tf2_ros
)
################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES foot_visualizer
 CATKIN_DEPENDS roscpp rospy std_msgs urdf xacro tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY src/meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  FILES_MATCHING PATTERN "*.stl"
)



add_executable(foot_visualizer src/main.cpp src/foot_visualizer.hpp)
add_executable(foot_tf_broadcaster src/tf.cpp)
target_link_libraries(foot_visualizer 
  ${catkin_LIBRARIES}
)
target_link_libraries(foot_tf_broadcaster 
  ${catkin_LIBRARIES}
)
