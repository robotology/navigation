#
# Copyright (C) 2010 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME forceGuidance)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${GSL_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source})

target_link_libraries(${PROJECTNAME} ${GSL_LIBRARIES} ${YARP_LIBRARIES})

set_property(TARGET ${PROJECTNAME} PROPERTY FOLDER "Modules")

install(TARGETS ${PROJECTNAME} DESTINATION bin)
