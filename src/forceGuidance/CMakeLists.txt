# SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(PROJECTNAME forceGuidance)
project(${PROJECTNAME})

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${ICUB_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} PUBLIC ctrlLib YARP::YARP_os YARP::YARP_dev YARP::YARP_math navigation_lib)

set_property(TARGET ${PROJECTNAME} PROPERTY FOLDER "Modules")

install(TARGETS ${PROJECTNAME} DESTINATION bin)

