cmake_minimum_required(VERSION 2.8.3)
project(temoto_context_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1y)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  message_generation
  geometry_msgs
  shape_msgs
  visualization_msgs
  diagnostic_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  temoto_core
  temoto_resource_registrar
)

add_message_files(FILES 
  ObjectContainer.msg
  MapContainer.msg
  ItemContainer.msg
  ComponentContainer.msg
  RobotContainer.msg
)

add_service_files(
  FILES
  UpdateEmr.srv
  GetEMRItem.srv
  GetEMRVector.srv
  RemoveItem.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
  diagnostic_msgs
  temoto_resource_registrar
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yaml-cpp
  CATKIN_DEPENDS roscpp roslib temoto_core temoto_resource_registrar
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(temoto_context_manager 
  src/context_manager_node.cpp
  src/context_manager.cpp
  src/context_manager_containers.cpp
  src/env_model_repository.cpp
  src/emr_ros_interface.cpp
)

add_dependencies(temoto_context_manager
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(temoto_context_manager
  ${catkin_LIBRARIES}
)

#
# Build the examples
#

# Setter example
add_executable(emr_item_setter_example 
  examples/set_emr_items.cpp
)

add_dependencies(emr_item_setter_example
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(emr_item_setter_example
  ${catkin_LIBRARIES}
)

# Visualizer example
add_executable(emr_item_visualizer_example 
  examples/visualize_emr_items.cpp
)

add_dependencies(emr_item_visualizer_example
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(emr_item_visualizer_example
  ${catkin_LIBRARIES}
)
