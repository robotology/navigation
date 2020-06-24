#
# Copyright (C) 2019 - IIT Istituto Italiano di Tecnologia 
# Author: Luca Nobile luca.nobile@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME headObstaclesScanner)
project(${PROJECTNAME})

# YARP needs CMake 3.0 or greater
cmake_minimum_required(VERSION 3.12)

# find YARP
find_package(YARP COMPONENTS os REQUIRED)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

find_package(YARP REQUIRED COMPONENTS sig cv dev os math rosmsg)
include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

