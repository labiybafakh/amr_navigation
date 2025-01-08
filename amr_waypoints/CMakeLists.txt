cmake_minimum_required(VERSION 3.0.2)
project(amr_waypoints)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
  tf2
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS actionlib move_base_msgs roscpp tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

