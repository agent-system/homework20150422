cmake_minimum_required(VERSION 2.8.3)
project(homework20150422)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  turtlesim
  cv_camera
  cv_bridge
  roseus
  roscpp
  image_transport)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES homework20150422
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(image_proc src/image_proc.cpp)
target_link_libraries(image_proc ${catkin_LIBRARIES})
add_dependencies(image_proc ${catkin_EXPORTED_TARGETS})