#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(odomLocalizer
                    CATEGORY device
                    TYPE odomLocalizer
                    INCLUDE odomLocalizer.h
                    DEFAULT ON)

if(SKIP_odomLocalizer)
   return()
endif()
                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(odomLocalizer odomLocalizer.h odomLocalizer.cpp)

include_directories (include)

target_link_libraries(odomLocalizer YARP::YARP_os
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   navigation_lib)


yarp_install(TARGETS odomLocalizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET odomLocalizer PROPERTY FOLDER "Plugins/Localization Devices")

