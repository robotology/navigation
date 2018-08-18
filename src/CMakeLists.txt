#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

add_subdirectory(baseControl)
add_subdirectory(forceGuidance)
add_subdirectory(robotGoto)
add_subdirectory(robotPathPlanner)
add_subdirectory(navigationGUI)
add_subdirectory(localizationServer)

yarp_begin_plugin_library(navmod OPTION NAV_COMPILE_DEVICE_PLUGINS
                                 DEFAULT ON
                                 DOC "devices plugins")
add_subdirectory(navigationServer)
add_subdirectory(rosNavigator)

yarp_end_plugin_library(navmod)

#add_subdirectory(mapper2D)


add_subdirectory(tests)
add_subdirectory(tutorials)
add_subdirectory(templates)
