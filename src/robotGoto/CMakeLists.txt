#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME robotGoto)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include (YarpIDL)
yarp_add_idl (ROS_MSG "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/visualization_msgs/MarkerArray.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/geometry_msgs/PoseStamped.msg")

include_directories(${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header} ${ROS_MSG})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

