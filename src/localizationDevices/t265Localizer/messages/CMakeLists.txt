# SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

#include(YarpChooseIDL)
#yarp_choose_idl(POZYX_MSG pozyxLocalizerMsgs.thrift)
yarp_add_idl(POZYX_MSG_GEN_FILES pozyxLocalizerMsgs.thrift)

add_library(pozyxLocalizerMsgs OBJECT ${POZYX_MSG_GEN_FILES})

#target_sources(pozyxLocalizerMsgs PRIVATE ${POZYX_MSG_THRIFT_GEN_FILES})

target_link_libraries(pozyxLocalizerMsgs
  PRIVATE
    YARP::YARP_dev
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_math
)

target_include_directories(pozyxLocalizerMsgs PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set_property(TARGET pozyxLocalizerMsgs PROPERTY FOLDER "Plugins/Shared")
