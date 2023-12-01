#
# Copyright (C) 2019 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

if(NAVIGATION_USE_ROS)
    add_subdirectory(rosLocalizer)
endif()
if(NAVIGATION_USE_ROS2)
    add_subdirectory(ros2Localizer)
endif()
add_subdirectory(odomLocalizer)
add_subdirectory(gazeboLocalizer)
add_subdirectory(pozyxLocalizer)
add_subdirectory(t265Localizer)
add_subdirectory(amclLocalizer)

