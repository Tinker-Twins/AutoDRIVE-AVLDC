# CMake configuration
cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0148 OLD)

# Project name
project(aeb_function)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  cmake_modules
)

# Catkin configuration
catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs tf
)

# Install Python executables
catkin_install_python(PROGRAMS
  scripts/opencav_aeb.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Add other directories
install(DIRECTORY launch rviz 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)