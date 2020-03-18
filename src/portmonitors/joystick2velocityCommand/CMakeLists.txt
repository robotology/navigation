# Copyright (C) 2016 iCub Facility, Istituto Italiano di Tecnologia
# Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(joy2vel TYPE Joy2vel
                                 INCLUDE joystick2velocityCommand.h
                                 CATEGORY portmonitor)

yarp_add_plugin(portmonitor_joy2vel
              joystick2velocityCommand.cpp
              joystick2velocityCommand.h)

target_link_libraries(portmonitor_joy2vel YARP::YARP_os
                                          YARP::YARP_dev
                                          YARP::YARP_init
                                          YARP::YARP_sig)

yarp_install(TARGETS portmonitor_joy2vel
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT Runtime
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

