cmake_minimum_required(VERSION 2.8.3)
project(robotnik_email)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  rcomponent
  robotnik_msgs
  #message_generation
)

#add_service_files(
#  FILES
#  SendAlarms.srv
#)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  robotnik_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    rcomponent
   # message_runtime
  DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  src/${PROJECT_NAME}/smtp_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Modify this if launch or config folders are deleted
install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
