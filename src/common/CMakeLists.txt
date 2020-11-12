# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECT_NAME navigation_lib)

set(LIBRARY_TARGET_NAME ${PROJECT_NAME})

set(${LIBRARY_TARGET_NAME}_SRC
        movable_localization_device/movable_localization_device.cpp
        odometry_estimation/localization_device_with_estimated_odometry.cpp
        recovery_behaviors/recovery_behaviors.cpp)


set(${LIBRARY_TARGET_NAME}_HDR
        movable_localization_device/movable_localization_device.h
        odometry_estimation/localization_device_with_estimated_odometry.h
        recovery_behaviors/recovery_behaviors.h
        include/navigation_defines.h)

add_library(${LIBRARY_TARGET_NAME} ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})
add_library(navigation::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION       ${navigation_VERSION}
                                                        SOVERSION     0
                                                        PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/recovery_behaviors>" 
                                                         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/movable_localization_device>"
                                                         "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                         "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries (${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_dev YARP::YARP_math ctrlLib)

install(TARGETS ${LIBRARY_TARGET_NAME}
        EXPORT  navigation
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT shlib
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                            COMPONENT lib
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                            COMPONENT bin
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}" COMPONENT dev)


message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")
