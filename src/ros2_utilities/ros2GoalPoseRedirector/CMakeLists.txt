#
# Copyright (C) 2016 iCub Facility - IIT Istituto Italiano di Tecnologia
# Author: Ettore Landini ettore.landini@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

project(ros2GoalPoseRedirector)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})


include_directories(${OpenCV_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES}
    rclcpp::rclcpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "Modules")
install(TARGETS ${PROJECT_NAME} DESTINATION bin)