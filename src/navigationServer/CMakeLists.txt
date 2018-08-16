yarp_prepare_plugin(navigation2DServer
                    CATEGORY device
                    TYPE navigation2DServer
                    INCLUDE navigation2DServer.h
                    INTERNAL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

yarp_add_plugin(navigation2DServer navigation2DServer.h
                                   navigation2DServer.cpp
                                  ${ROS_MSG})
                              
target_link_libraries(navigation2DServer YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   YARP::YARP_rosmsg
                                   ${OpenCV_LIBRARIES})

yarp_install(TARGETS navigation2DServer
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES navigation2DServer.ini
           COMPONENT ${YARP_PLUGIN_MASTER}
           DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET navigation2DServer PROPERTY FOLDER "Plugins/Device")
