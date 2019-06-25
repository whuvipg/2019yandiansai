########### 版本要求
cmake_minimum_required(VERSION 2.8.3)

########### 项目名称
project(ugv_control)

########### 添加系统库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  serial
  tf
  nav_msgs
  actionlib
  move_base_msgs
)

catkin_package(CATKIN_DEPENDS
  sensor_msgs
  geometry_msgs
  tf
  nav_msgs
  actionlib
  move_base_msgs
)

########### 添加头文件
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(ugvdriver_node src/ugvdriver_node.cpp src/ugvdriver.cpp src/serialtransmission.cpp src/coordinatetransform.cpp)
target_link_libraries(ugvdriver_node ${catkin_LIBRARIES})

add_executable(keyboardcontrol_node src/keyboardcontrol_node.cpp)
target_link_libraries(keyboardcontrol_node ${catkin_LIBRARIES})

add_executable(path_node src/path_node.cpp src/path.cpp)
target_link_libraries(path_node ${catkin_LIBRARIES})

add_executable(goal_test_node src/goal_test_node.cpp)
target_link_libraries(goal_test_node ${catkin_LIBRARIES})