#
# Copyright (C) 2017 IIT - iCubFacility
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(fakeMobileBaseTest)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${GSL_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${folder_source} ${folder_header} ${ROS_MSG})

target_link_libraries(${PROJECT_NAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

