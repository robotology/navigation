#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

add_subdirectory(common)

# add_subdirectory(deprecated)

add_subdirectory(baseControl)
add_subdirectory(forceGuidance)
add_subdirectory(navigationGUI)

yarp_begin_plugin_library(navmod OPTION NAV_COMPILE_DEVICE_PLUGINS
                                 DEFAULT ON
                                 DOC "devices plugins")
add_subdirectory(localizationDevices)
add_subdirectory(navigationDevices)
yarp_end_plugin_library(navmod)

add_subdirectory(map2Gazebo)
#add_subdirectory(mapper2D)

add_subdirectory(follower)

add_subdirectory(portmonitors)
add_subdirectory(tests)
add_subdirectory(tutorials)
add_subdirectory(freeFloorViewer)

#add_subdirectory(extendedRangefinder2DWrapper)  da mettere???

