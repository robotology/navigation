#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(areas)

#file(GLOB folder_source areas.cpp)
file(GLOB folder_source roomsSegmentation2.cpp)

file(GLOB folder_header *.h)

find_package(YARP 3.9 COMPONENTS os sig dev math gsl idl_tools REQUIRED)
find_package(OpenCV)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
