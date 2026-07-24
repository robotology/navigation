# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_add_idl(GAZEBO_MSG_GEN_FILES gazeboLocalizerMsgs.thrift)

add_library(gazeboLocalizerMsgs OBJECT ${GAZEBO_MSG_GEN_FILES})

target_link_libraries(gazeboLocalizerMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
)

target_include_directories(gazeboLocalizerMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET gazeboLocalizerMsgs PROPERTY FOLDER "Plugins/Shared")
