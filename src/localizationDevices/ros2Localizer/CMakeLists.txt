#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

yarp_prepare_plugin(ros2Localizer
                    CATEGORY device
                    TYPE ros2Localizer
                    INCLUDE ros2Localizer.h
                    INTERNAL)


                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(ros2Localizer ros2Localizer.h ros2Localizer.cpp)
                              
target_link_libraries(ros2Localizer YARP::YARP_os
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   YARP::YARP_rosmsg
                                   navigation_lib
                                   nav_msgs::nav_msgs__rosidl_typesupport_cpp
                                   geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                                   tf2_msgs::tf2_msgs__rosidl_typesupport_cpp)


yarp_install(TARGETS ros2Localizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET ros2Localizer PROPERTY FOLDER "Plugins/Localization Devices")

