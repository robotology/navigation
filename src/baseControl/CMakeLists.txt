#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

include(YarpIDL)

#yarp_idl_to_dir("${CMAKE_CURRENT_SOURCE_DIR}/messages/nav_msgs/Odometry.msg"
#                "${CMAKE_CURRENT_SOURCE_DIR}"
#                ros_msg_sources
#                ros_msg_headers
#                ros_msg_include_dirs)
include(${CMAKE_CURRENT_SOURCE_DIR}/nav_msgs_Odometry_msg.cmake)

#yarp_idl_to_dir("${CMAKE_CURRENT_SOURCE_DIR}/messages/geometry_msgs/PolygonStamped.msg"
#                "${CMAKE_CURRENT_SOURCE_DIR}"
#                ros_msg_sources
#                ros_msg_headers
#                ros_msg_include_dirs)
include(${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs_PolygonStamped_msg.cmake)


#yarp_idl_to_dir("${CMAKE_CURRENT_SOURCE_DIR}/messages/geometry_msgs/TransformStamped.msg"
#                "${CMAKE_CURRENT_SOURCE_DIR}"
#                ros_msg_sources
#                ros_msg_headers
#                ros_msg_include_dirs)
include(${CMAKE_CURRENT_SOURCE_DIR}/geometry_msgs_TransformStamped_msg.cmake)

#yarp_idl_to_dir("${CMAKE_CURRENT_SOURCE_DIR}/messages/tf/tfMessage.msg"
#                "${CMAKE_CURRENT_SOURCE_DIR}"
#                ros_msg_sources
#                ros_msg_headers
#                ros_msg_include_dirs)
include(${CMAKE_CURRENT_SOURCE_DIR}/tf_tfMessage_msg.cmake)

set(PROJECTNAME baseControl)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp cer/*.cpp ikart/*.cpp)
file(GLOB folder_header *.h cer/*.h ikart/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${GSL_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

