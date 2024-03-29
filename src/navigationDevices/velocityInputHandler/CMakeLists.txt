yarp_prepare_plugin(velocityInputHandler
                    CATEGORY device
                    TYPE VelocityInputHandler
                    INCLUDE velocityInputHandler.h
                    DEFAULT ON)

if(SKIP_velocityInputHandler)
   return()
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

yarp_add_plugin(velocityInputHandler velocityInputHandler.h
                                     velocityInputHandler.cpp)
                              
target_link_libraries(velocityInputHandler YARP::YARP_os
                                   YARP::YARP_sig
                                   YARP::YARP_dev
                                   YARP::YARP_math
                                   navigation_lib)

yarp_install(TARGETS velocityInputHandler
           EXPORT YARP_${YARP_PLUGIN_MASTER}
           COMPONENT ${YARP_PLUGIN_MASTER}
           LIBRARY DESTINATION ${NAVIGATION_DYNAMIC_PLUGINS_INSTALL_DIR}
           ARCHIVE DESTINATION ${NAVIGATION_STATIC_PLUGINS_INSTALL_DIR}
           YARP_INI DESTINATION ${NAVIGATION_PLUGIN_MANIFESTS_INSTALL_DIR})


set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

set_property(TARGET velocityInputHandler PROPERTY FOLDER "Plugins/Navigation Devices")
