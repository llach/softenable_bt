cmake_minimum_required(VERSION 3.8)
project(softenable_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)

################
########    SKILLS LIBRARY
################
file(GLOB_RECURSE SKILL_SOURCES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  src/*/*.cpp
)

# Create the treenode library
add_library(softenable_skills ${SKILL_SOURCES})

target_include_directories(softenable_skills PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(softenable_skills
  behaviortree_cpp
)

################
########    EXECUTABLES
################

add_executable(grasp_first_layer src/grasp_first_layer.cpp)
target_link_libraries(grasp_first_layer softenable_skills)
ament_target_dependencies(grasp_first_layer rclcpp behaviortree_cpp)

install(TARGETS
  grasp_first_layer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()