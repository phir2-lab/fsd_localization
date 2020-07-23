cmake_minimum_required(VERSION 2.8.3)
project(fsd_localization)

find_package(
  catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  tf2_sensor_msgs
)

add_compile_options(-std=c++0x)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  roslib
  tf2_sensor_msgs
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
)

add_executable(fsd_localization
  src/main.cpp
  src/Utils.cpp
  src/GlutClass.cpp
  src/Grid.cpp
  src/DistanceTransform.cpp
  src/MCL.cpp
  src/Robot.cpp
  src/RobotBase.cpp
  src/RobotBase_ROS.cpp
)

target_link_libraries(fsd_localization
  ${catkin_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  freeimage
)

