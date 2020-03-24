cmake_minimum_required(VERSION 2.8.3)
project(prizm_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs

  cv_bridge
  image_transport
  sensor_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
generate_dynamic_reconfigure_options(cfg/StopOnWhite.cfg cfg/LineFollow.cfg)
#generate_dynamic_reconfigure_options(cfg/LineFollow.cfg)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
  include
)

# Compile prizm control
add_executable(prizm_control_node src/prizm_control_node.cpp)
target_link_libraries(prizm_control_node ${catkin_LIBRARIES})

add_executable(TPcontrol_joy_node src/TPcontrol_joy_node.cpp)
target_link_libraries(TPcontrol_joy_node ${catkin_LIBRARIES})

#add_executable(joy_control_node src/joy_control_node.cpp)
#target_link_libraries(joy_control_node ${catkin_LIBRARIES})

add_executable(stop_on_white src/stop_on_white.cpp)
target_link_libraries(stop_on_white ${catkin_LIBRARIES})
target_link_libraries(stop_on_white ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(line_follow src/line_follow.cpp)
target_link_libraries(line_follow ${catkin_LIBRARIES})
target_link_libraries(line_follow ${catkin_LIBRARIES} ${OpenCV_LIBS})
