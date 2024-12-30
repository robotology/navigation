/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Ros2Utils.h"


rclcpp::Node::SharedPtr NodeCreator::createNode(std::string name)
{
    if(!rclcpp::ok())
    {
        rclcpp::init(/*argc*/ 0, /*argv*/ nullptr);
    }
    return std::make_shared<rclcpp::Node>(name);

}
