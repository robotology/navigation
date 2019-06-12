#
#    Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#    All rights reserved.
#
#    This software may be modified and distributed under the terms of the
#    GPL-2+ license. See the accompanying LICENSE file for details.
#

yarp_prepare_plugin(rosNavigator
                    CATEGORY device
                    TYPE rosNavigator
                    INCLUDE rosNavigator.h
                    INTERNAL)

yarp_add_idl (ROS_MSG "${CMAKE_CURRENT_SOURCE_DIR}/../../ros_messages/actionlib_msgs/GoalID.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../../ros_messages/actionlib_msgs/GoalStatusArray.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../../ros_messages/move_base_msgs/MoveBaseActionGoal.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../../ros_messages/move_base_msgs/MoveBaseActionFeedback.msg"
                      "${CMAKE_CURRENT_SOURCE_DIR}/../../ros_messages/move_base_msgs/MoveBaseActionResult.msg")
                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(rosNavigator rosNavigator.h
                             rosNavigator.cpp
                             ${ROS_MSG})
                              
target_link_libraries(rosNavigator YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   YARP::YARP_rosmsg)

yarp_install(TARGETS rosNavigator
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET rosNavigator PROPERTY FOLDER "Plugins/Device")
