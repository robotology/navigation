#
# Copyright (C) 2018 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(pozyxLocalizer
                    CATEGORY device
                    TYPE pozyxLocalizer
                    INCLUDE pozyxLocalizer.h
                    INTERNAL)

                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(pozyxLocalizer pozyxLocalizer.h pozyxLocalizer.cpp)

include_directories (include)

target_link_libraries(pozyxLocalizer YARP::YARP_os
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   YARP::YARP_rosmsg
                                   navigation_lib)


yarp_install(TARGETS pozyxLocalizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
        
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET pozyxLocalizer PROPERTY FOLDER "Plugins/Localization Devices")

