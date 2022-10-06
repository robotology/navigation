# Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(joy2vel
                    CATEGORY portmonitor
                    TYPE Joy2vel
                    INCLUDE joystick2velocityCommand.h
                    INTERNAL)

if(SKIP_joy2vel)
  return()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(joy2vel
              joystick2velocityCommand.cpp
              joystick2velocityCommand.h)

include_directories (include)

target_link_libraries(joy2vel YARP::YARP_os
                                          YARP::YARP_dev
                                          YARP::YARP_init
                                          YARP::YARP_sig)

yarp_install(TARGETS joy2vel
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET joy2vel PROPERTY FOLDER "Plugins/Localization Devices")
