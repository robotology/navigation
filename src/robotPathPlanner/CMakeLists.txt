#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME robotPathPlanner)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)

