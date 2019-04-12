#
#    Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
#    All rights reserved.
# 
#    This software may be modified and distributed under the terms of the
#    GPL-2+ license. See the accompanying LICENSE file for details.
#

yarp_prepare_plugin(t265Localizer
                    CATEGORY device
                    TYPE t265Localizer
                    INCLUDE t265Localizer.h
                    INTERNAL)

                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(t265Localizer t265Localizer.h t265Localizer.cpp)
                              
target_link_libraries(t265Localizer YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   YARP::YARP_rosmsg)


yarp_install(TARGETS t265Localizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES t265Localizer.ini
           COMPONENT ${YARP_PLUGIN_MASTER}
           DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET t265Localizer PROPERTY FOLDER "Plugins/Device")

