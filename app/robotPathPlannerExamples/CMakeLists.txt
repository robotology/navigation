#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

yarp_install(FILES ${conf}    DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/robotPathPlannerExamples)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})

