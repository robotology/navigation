#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

yarp_prepare_plugin(ros2Navigator
    CATEGORY device
    TYPE ros2Navigator
    INCLUDE ros2Navigator.h
    INTERNAL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

yarp_add_plugin(ros2Navigator ${folder_source} ${folder_header})

target_link_libraries(ros2Navigator YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    nav2_msgs::nav2_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    navigation_lib)

yarp_install(TARGETS ros2Navigator
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET ros2Navigator PROPERTY FOLDER "Plugins/Navigation Devices")