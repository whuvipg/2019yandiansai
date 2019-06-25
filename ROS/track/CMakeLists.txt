cmake_minimum_required(VERSION 2.8)
project(track)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  cv_bridge
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision
  CATKIN_DEPENDS geometry_msgs  
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

########### 编译可执行文件
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})	
