# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_add_idl(AMCL_MSG_GEN_FILES amclLocalizerMsgs.thrift)

add_library(amclLocalizerMsgs OBJECT ${AMCL_MSG_GEN_FILES})

target_link_libraries(amclLocalizerMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
)

target_include_directories(amclLocalizerMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET amclLocalizerMsgs PROPERTY FOLDER "Plugins/Shared")
