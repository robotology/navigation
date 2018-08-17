#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname rosExamples)

file(GLOB conf      ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
file(GLOB templates ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)
file(GLOB apps      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${NAVIGATION_CONTEXTS_INSTALL_DIR}/${appname})
yarp_install(FILES ${apps}    DESTINATION ${NAVIGATION_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES ${scripts} DESTINATION ${NAVIGATION_APPLICATIONS_TEMPLATES_INSTALL_DIR})

