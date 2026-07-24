# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_add_idl(ROBOGOTO_MSG_GEN_FILES robotGotoMsgs.thrift)

add_library(robotGotoMsgs OBJECT ${ROBOGOTO_MSG_GEN_FILES})

target_link_libraries(robotGotoMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
)

target_include_directories(robotGotoMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET robotGotoMsgs PROPERTY FOLDER "Plugins/Shared")
