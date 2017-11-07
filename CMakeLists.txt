#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia 
# Author: Marco Randazzo marco.randazzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

cmake_minimum_required(VERSION 2.8.12)

project(navigation)
if(WIN32)
    #disable annoying warnings about use of deprecated c functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    add_definitions(/wd4250)
endif()

# Enable C++11
include(CheckCXXCompilerFlag)
unset(CXX11_FLAGS)
check_cxx_compiler_flag("-std=c++11" CXX_HAS_STD_CXX11)
if(CXX_HAS_STD_CXX11)
  set(CXX11_FLAGS "-std=c++11")
endif()

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED 11)
if(NOT CMAKE_MINIMUM_REQUIRED_VERSION VERSION_LESS 3.1)
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(${CMAKE_VERSION} VERSION_LESS 3.1)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAGS}")
endif()

#### Find YARP, require that YARP version is at least ICUB_REQYARP_VERSION
set(ICUB_REQYARP_VERSION_MAJOR "2")
set(ICUB_REQYARP_VERSION_MINOR "3")
set(ICUB_REQYARP_VERSION_PATCH "71")
set(ICUB_REQYARP_VERSION_TWEAK "")
if(ICUB_REQYARP_VERSION_TWEAK)
    set(ICUB_REQYARP_VERSION ${ICUB_REQYARP_VERSION_MAJOR}.${ICUB_REQYARP_VERSION_MINOR}.${ICUB_REQYARP_VERSION_PATCH}.${ICUB_REQYARP_VERSION_TWEAK})
else()
    set(ICUB_REQYARP_VERSION ${ICUB_REQYARP_VERSION_MAJOR}.${ICUB_REQYARP_VERSION_MINOR}.${ICUB_REQYARP_VERSION_PATCH})
endif()
find_package(YARP ${ICUB_REQYARP_VERSION} REQUIRED)
message(STATUS "YARP is version: ${YARP_VERSION}")

#### Find iCub 
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

find_package(OpenCV)
find_package(GSL)
find_package(SDL)

add_subdirectory(src)
add_subdirectory(app)

icubcontrib_add_uninstall_target()

