cmake_minimum_required(VERSION 2.8.3)
project(homework20150422)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  turtlesim
  cv_camera
  cv_bridge
  image_transport
  image_view2
  roseus)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES homework20150422
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(mean src/mean.cpp)
target_link_libraries(mean ${catkin_LIBRARIES})
