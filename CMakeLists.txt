cmake_minimum_required(VERSION 2.8.3)
project(carla_to_ros)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    rospy
    #sensor_msgs
    geometry_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
  CATKIN_DEPENDS
    #sensor_msgs
    geometry_msgs
#  DEPENDS
)

#[[install(PROGRAMS 
  src/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
]]