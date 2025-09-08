cmake_minimum_required(VERSION 3.8)
project(softenable_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stack_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

################
########    SKILLS LIBRARY
################
file(GLOB_RECURSE SKILL_SOURCES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  src/*/*.cpp
)
message(STATUS "SKILL_SOURCES = ${SKILL_SOURCES}")

# Create the treenode library
add_library(softenable_skills ${SKILL_SOURCES})

target_include_directories(softenable_skills PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(softenable_skills
  rclcpp
  std_srvs
  stack_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
)

################
########    EXECUTABLES
################

add_executable(grasp_first_layer src/grasp_first_layer.cpp)
target_link_libraries(grasp_first_layer softenable_skills)
ament_target_dependencies(grasp_first_layer 
  rclcpp 
  behaviortree_cpp 
  geometry_msgs
  stack_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(test_nodes src/test_nodes.cpp)
target_link_libraries(test_nodes softenable_skills)
ament_target_dependencies(test_nodes 
  rclcpp 
  behaviortree_cpp 
  geometry_msgs
  stack_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)


install(TARGETS
  grasp_first_layer test_nodes
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()