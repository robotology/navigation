yarp_prepare_plugin(navigationDeviceTemplate
                    CATEGORY device
                    TYPE navigationDeviceTemplate
                    INCLUDE navigationDeviceTemplate.h
                    INTERNAL)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(navigationDeviceTemplate navigationDeviceTemplate.h
                             navigationDeviceTemplate.cpp)
                              
target_link_libraries(navigationDeviceTemplate YARP::YARP_OS
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math)

yarp_install(TARGETS navigationDeviceTemplate
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})

set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET navigationDeviceTemplate PROPERTY FOLDER "Plugins/Device")
