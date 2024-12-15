cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf2_ros
  move_base
  amcl
  map_server
  teb_local_planner
  base_local_planner
  dwa_local_planner
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
    geometry_msgs 
    nav_msgs 
    tf2_ros
    move_base
    amcl
    map_server
    teb_local_planner
    base_local_planner
    dwa_local_planner
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/cmdvel_to_odom.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY config launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)