#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Ettore Landini ettore.landini@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(freeFloorViewer)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package(YARP REQUIRED COMPONENTS sig cv dev os math rosmsg)
include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES} navigation_lib)
set_property(TARGET freeFloorViewer PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

