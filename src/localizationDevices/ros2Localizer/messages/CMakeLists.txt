# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

#include(YarpChooseIDL)
#yarp_choose_idl(POZYX_MSG ros2LocalizerMsgs.thrift)
yarp_add_idl(POZYX_MSG_GEN_FILES ros2LocalizerMsgs.thrift)

add_library(ros2LocalizerMsgs OBJECT ${POZYX_MSG_GEN_FILES})

#target_sources(ros2LocalizerMsgs PRIVATE ${POZYX_MSG_THRIFT_GEN_FILES})

target_link_libraries(ros2LocalizerMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_math
)

target_include_directories(ros2LocalizerMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET ros2LocalizerMsgs PROPERTY FOLDER "Plugins/Shared")
