# Copyright (C) 2026 iCub Facility, Istituto Italiano di Tecnologia
# CopyPolicy: LGPLv2.1+

set(CMAKE_INCLUDE_CURRENT_DIR TRUE)

yarp_prepare_plugin(joyNwsYarp2vel
                    CATEGORY portmonitor
                    TYPE JoyNwsYarp2vel
                    INCLUDE joystickNwsYarp2velocityCommand.h
                    INTERNAL)

if(SKIP_joy2vel)
  return()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(joyNwsYarp2vel
              joystickNwsYarp2velocityCommand.cpp
              joystickNwsYarp2velocityCommand.h)

get_filename_component(ROBOT_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)

include_directories (include
                     "${ROBOT_SRC_DIR}/yarp/src/devices/messages/IJoypadControlMsg/idl_generated_code"
                     "${ROBOT_SRC_DIR}/yarp/src/libYARP_dev/src/idl_generated_code")

target_link_libraries(joyNwsYarp2vel YARP::YARP_os
                                          YARP::YARP_dev
                                          YARP::YARP_init
                                          YARP::YARP_sig)

yarp_install(TARGETS joyNwsYarp2vel
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET joyNwsYarp2vel PROPERTY FOLDER "Plugins/Portmonitors")
