#
# Copyright (C) 2017 RobotCub Consortium
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(PROJECTNAME baseControl)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp cer/*.cpp ikart/*.cpp)
file(GLOB folder_header *.h cer/*.h ikart/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${GSL_INCLUDE_DIRS}
                    ${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ctrlLib ${GSL_LIBRARIES} ${YARP_LIBRARIES} navigation_lib)

set_property(TARGET ${PROJECTNAME} PROPERTY FOLDER "Modules")

install(TARGETS ${PROJECTNAME} DESTINATION bin)

