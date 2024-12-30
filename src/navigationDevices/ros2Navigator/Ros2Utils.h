/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef YARP_ROS2_ROS2UTILS_H
#define YARP_ROS2_ROS2UTILS_H

#include <rclcpp/rclcpp.hpp>
class NodeCreator
{
public:
	static rclcpp::Node::SharedPtr createNode(std::string name);
};


#endif // YARP_ROS2_ROS2UTILS_H
