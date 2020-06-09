cmake_minimum_required(VERSION 3.0.2)
project(panda_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  controller_manager
  effort_controllers
  gazebo_ros
  gazebo_plugins
  joint_state_controller
  joint_state_publisher
  joint_trajectory_controller
  robot_state_publisher
  std_msgs
  tf
  xacro
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  controller_interface
  hardware_interface
  message_generation
)

find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  VacuumGripperState.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  VacuumGripperControl.srv
  CartesianGoal.srv
  JointsGoal.srv
  NamedGoal.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy 
  moveit_core 
  moveit_visual_tools 
  moveit_ros_planning_interface
  hardware_interface
  pluginlib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(robot_state_initializer_node src/robot_state_initializer.cpp)

target_link_libraries(robot_state_initializer_node ${catkin_LIBRARIES})

# add custom controller as library
add_library(${PROJECT_NAME}_controllers_lib src/joint_position_controller.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_controllers_lib ${catkin_LIBRARIES})

## Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/VacuumGripperPlugin.cc)
target_link_libraries(${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

## Create the libROSVacuumGripperPlugin.so library.
set(ros_vacuum_gripper_plugin_name ROSVacuumGripperPlugin)
add_library(${ros_vacuum_gripper_plugin_name} src/ROSVacuumGripperPlugin.cc)
target_link_libraries(${ros_vacuum_gripper_plugin_name}
  ${catkin_LIBRARIES}
  ${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_vacuum_gripper_plugin_name}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
 
