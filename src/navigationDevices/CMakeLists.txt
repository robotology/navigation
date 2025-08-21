#
# Copyright (C) 2019 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

add_subdirectory(navigationDeviceTemplate)
add_subdirectory(robotGotoDevice)
add_subdirectory(robotPathPlannerDevice)
if(NAVIGATION_USE_ROS2)
    add_subdirectory(ros2Navigator)
endif()
add_subdirectory(simpleVelocityNavigation)
add_subdirectory(velocityInputHandler)
