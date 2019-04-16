#
# Copyright (C) 2019 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

yarp_prepare_plugin(isaacLocalizer
                    CATEGORY device
                    TYPE isaacLocalizer
                    INCLUDE isaacLocalizer.h
                    INTERNAL)
                                            
                
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

yarp_add_plugin(isaacLocalizer isaacLocalizer.h isaacLocalizer.cpp
           #     /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/external/asio/asio/include/asio.hpp
                )
                              
target_link_libraries(isaacLocalizer YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                    /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin/engine/alice/libimpl.pic.a
                              #       /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin/engine/alice/libimpl.a
                                   -L/home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin/engine/alice
                                   #-L/home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin/messages
                                   #-L/home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin
                                   /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/bin/external/asio/_objs/asio/asio.pic.o
                                   /usr/local/src/robot/isaac_sdk/bazel-out/k8-opt/bin/engine/gems/scheduler/libscheduler.pic.a
                                   /usr/local/src/robot/isaac_sdk/bazel-out/k8-opt/bin/engine/core/libcore.pic.a)

target_include_directories (isaacLocalizer PRIVATE /usr/local/src/robot/isaac_sdk
                                                   /usr/include/eigen3
                                                   /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/external/capnproto/c++/src
                                                   /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/execroot/isaac/bazel-out/k8-opt/genfiles
                                                   /home/r1-user/.cache/bazel/_bazel_r1-user/650aaf1a95e1d659eb66df38aaeaa4a4/external/asio/asio/include)

yarp_install(TARGETS isaacLocalizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES isaacLocalizer.ini
           COMPONENT ${YARP_PLUGIN_MASTER}
           DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET isaacLocalizer PROPERTY FOLDER "Plugins/Device")

