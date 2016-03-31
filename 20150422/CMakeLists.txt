cmake_minimum_required(VERSION 2.8.3)
project(homework20150422)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  turtlesim
  cv_camera
  cv_bridge
  usb_cam
  pcl_ros
  sensor_msgs
  roseus)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES homework20150422
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

 add_executable(pub_pseudo_organized_image_points2 src/pub_pseudo_organized_image_points2.cpp)

target_link_libraries(pub_pseudo_organized_image_points2
  ${catkin_LIBRARIES}
)
