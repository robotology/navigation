#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

# add_subdirectory(deprecated)

add_subdirectory(baseControl)
add_subdirectory(forceGuidance)
add_subdirectory(navigationGUI)

yarp_begin_plugin_library(navmod OPTION NAV_COMPILE_DEVICE_PLUGINS
                                 DEFAULT ON
                                 DOC "devices plugins")
add_subdirectory(rosNavigator)
add_subdirectory(rosLocalizer)
add_subdirectory(odomLocalizer)
add_subdirectory(pozyxLocalizer)
#add_subdirectory(amclLocalizer)
add_subdirectory(robotGotoDevice)
add_subdirectory(robotPathPlannerDevice)
add_subdirectory(map2Gazebo)

yarp_end_plugin_library(navmod)

#add_subdirectory(mapper2D)


add_subdirectory(tests)
add_subdirectory(tutorials)
add_subdirectory(templates)
