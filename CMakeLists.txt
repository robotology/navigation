#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 3.5)

project(navigation)

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

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)

#### Find YARP, require that YARP version is at least ICUB_REQYARP_VERSION
find_package(YARP 3.3.101 COMPONENTS os sig dev math gsl rosmsg idl_tools REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

yarp_configure_external_installation(${PROJECT_NAME} WITH_PLUGINS)

#### Find iCub 
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(OpenCV)
find_package(GSL)
find_package(SDL)

add_subdirectory(src)
add_subdirectory(app)

#icubcontrib_add_uninstall_target()
