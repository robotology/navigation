
# SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(PROJECTNAME baseControl2)
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

