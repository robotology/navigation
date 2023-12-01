#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(robotPathPlannerDev
                    CATEGORY device
                    TYPE robotPathPlannerDev
                    INCLUDE robotPathPlannerDev.h
                    DEFAULT ON)

if(SKIP_robotPathPlannerDev)
   return()
endif()


set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(robotPathPlannerDev robotPathPlannerDev.h robotPathPlannerDev.cpp
                pathPlannerCtrl.cpp pathPlannerCtrl.h
                pathPlannerCtrlActions.cpp pathPlannerCtrlGets.cpp pathPlannerCtrlInit.cpp
                pathPlannerCtrlHelpers.cpp pathPlannerCtrlHelpers.h)

target_link_libraries(robotPathPlannerDev YARP::YARP_os
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   navigation_lib
                                   ${OpenCV_LIBRARIES})


yarp_install(TARGETS robotPathPlannerDev
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET robotPathPlannerDev PROPERTY FOLDER "Plugins/Navigation Devices")

