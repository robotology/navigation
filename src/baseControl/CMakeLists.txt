#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME baseControl)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp cer/*.cpp ikart/*.cpp)
file(GLOB folder_header *.h cer/*.h ikart/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include (YarpIDL)
yarp_add_idl (ROS_MSG "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/visualization_msgs/MarkerArray.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/nav_msgs/Odometry.msg"
					  "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/geometry_msgs/PolygonStamped.msg"
					  "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/geometry_msgs/TransformStamped.msg"
					  "${CMAKE_CURRENT_SOURCE_DIR}/../ros_messages/tf/tfMessage.msg")

include_directories(${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header} ${ROS_MSG})

target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

