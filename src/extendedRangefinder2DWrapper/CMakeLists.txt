#
# Copyright (C) 2019 - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(extendedRangefinder2DWrapper
                    CATEGORY device
                    TYPE extendedRangefinder2DWrapper
                    INCLUDE extendedRangefinder2DWrapper.h
                    INTERNAL)



set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(extendedRangefinder2DWrapper extendedRangefinder2DWrapper.cpp
                                                  extendedRangefinder2DWrapper.h)





target_link_libraries(extendedRangefinder2DWrapper  YARP::YARP_os
                                                    YARP::YARP_sig
                                                    YARP::YARP_dev
                                                    YARP::YARP_math
                                                    YARP::YARP_rosmsg)

yarp_install(TARGETS extendedRangefinder2DWrapper
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET extendedRangefinder2DWrapper PROPERTY FOLDER "Plugins/Device")

