cmake_minimum_required(VERSION 2.8.3)
project(prizm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compile prizm control
add_executable(prizm_control_node src/prizm_control_node.cpp)
target_link_libraries(prizm_control_node ${catkin_LIBRARIES})
