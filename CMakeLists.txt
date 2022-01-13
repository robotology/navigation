# Copyright (C) 2021 IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12)

set(ROOT_PROJECT_NAME navigation)
project(${ROOT_PROJECT_NAME} LANGUAGES C CXX
                             VERSION 1.19.2)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(YCM_REQUIRED_VERSION 0.11.0)
set(YARP_REQUIRED_VERSION 3.5)

find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)

option(NAVIGATION_USE_ROS2 "if enabled, ROS2 related executables/plugins will be compiled" OFF)

option(NAVIGATION_USE_NWC "if enabled, yarp nwc devices instead of standard clients" OFF)
if(NAVIGATION_USE_NWC)
    add_definitions(-DNAVIGATION_USE_NWC=1)
endif()

if(YARP_math_FOUND)
  set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
  message(STATUS "found libYARP_math")
else()
  set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
  message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
endif()

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_SHARED_LIBS ON)

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)

find_package(YCM REQUIRED)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        message(STATUS "Setting build type to 'Release' as none was specified.")
        set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

#### Find YARP, require that YARP version is at least ICUB_REQYARP_VERSION
find_package(YARP 3.5 COMPONENTS os sig dev math gsl rosmsg idl_tools REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

yarp_configure_external_installation(${PROJECT_NAME} WITH_PLUGINS)

#### Find iCub 
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(OpenCV)
find_package(GSL)
find_package(SDL)

if(NAVIGATION_USE_ROS2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_msgs REQUIRED)
    find_package(test_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
endif()

add_subdirectory(src)
add_subdirectory(app)

set_property(GLOBAL PROPERTY USE_FOLDERS 1)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            NAMESPACE navigation::
                            VERSION ${${PROJECT_NAME}_VERSION}
                            EXPORT navigation
                            COMPATIBILITY AnyNewerVersion
                            DEPENDENCIES ICUB
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)
include(AddUninstallTarget)

