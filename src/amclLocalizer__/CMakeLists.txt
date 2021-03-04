#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
yarp_prepare_plugin(amclLocalizer
                    CATEGORY device
                    TYPE amclLocalizer
                    INCLUDE amclLocalizer.h
                    INTERNAL)


                                            
set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(amclLocalizer amclLocalizer.h amclLocalizer.cpp
                amcl/sensors/amcl_laser.cpp
                amcl/sensors/amcl_odom.cpp
                amcl/sensors/amcl_sensor.cpp
                amcl/sensors/amcl_laser.h
                amcl/sensors/amcl_odom.h
                amcl/sensors/amcl_sensor.h
                amcl/pf/eig3.c
                amcl/pf/pf.c
                amcl/pf/pf_draw.c
                amcl/pf/pf_kdtree.c
                amcl/pf/pf_pdf.c
                amcl/pf/pf_vector.c
                amcl/pf/eig3.h
                amcl/pf/pf.h
                amcl/pf/pf_kdtree.h
                amcl/pf/pf_pdf.h
                amcl/pf/pf_vector.h
                amcl/map/map.c
                amcl/map/map_cspace.cpp
                amcl/map/map_range.c
                amcl/map/map_store.c
                amcl/map/map.h)
                              
target_link_libraries(amclLocalizer YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math)


yarp_install(TARGETS amclLocalizer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES amclLocalizer.ini
           COMPONENT ${YARP_PLUGIN_MASTER}
           DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})
           
set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET amclLocalizer PROPERTY FOLDER "Plugins/Device")

