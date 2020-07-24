cmake_minimum_required(VERSION 2.8.3)
project(tb3_exame)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  roscpp
  rospy
  std_msgs
  turtlebot3_gazebo
  turtlebot3_msgs
  amcl
  std_srvs
  geometry_msgs
)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

  CATKIN_DEPENDS 
  move_base_msgs 
  roscpp 
  rospy 
  std_msgs 
  turtlebot3_gazebo
  turtlebot3_msgs 
  amcl 
  std_srvs
  geometry_msgs
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
 include  
 ${catkin_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
)


add_executable(navi src/navigation.cpp)
add_executable(navi_amcl src/navigation_amcl.cpp)
add_executable(simu src/simulation.cpp)
add_executable(simu2 src/simulation2.cpp)

add_dependencies(navi ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(navi_amcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(simu2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(navi ${catkin_LIBRARIES})
target_link_libraries(navi_amcl ${catkin_LIBRARIES})
target_link_libraries(simu ${catkin_LIBRARIES})
target_link_libraries(simu2 ${catkin_LIBRARIES})

