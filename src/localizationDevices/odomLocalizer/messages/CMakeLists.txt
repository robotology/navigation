# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_add_idl(ODOM_MSG_GEN_FILES odomLocalizerMsgs.thrift)

add_library(odomLocalizerMsgs OBJECT ${ODOM_MSG_GEN_FILES})

target_link_libraries(odomLocalizerMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_math
)

target_include_directories(odomLocalizerMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET odomLocalizerMsgs PROPERTY FOLDER "Plugins/Shared")
