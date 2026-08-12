/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ROS2GOALPOSEREDIRECTOR_H
#define ROS2GOALPOSEREDIRECTOR_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Log.h>
#include <yarp/math/Quaternion.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/Map2DLocationData.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


/**
 * @ingroup
 *
 * @brief This class is a YARP RFModule that redirects goal poses from a ROS2 topic to a YARP INavigation2D interface.
 * \section Parameters
 * |Group             |Parameter          |Type    |Units|Default                        |Description                                       |
 * |------------------|-------------------|--------|-----|-------------------------------|--------------------------------------------------|
 * |NAVIGATION_CLIENT |device             | string |     |                               |Device name of the INavigation2D interface        |
 * |NAVIGATION_CLIENT |local              | string |     |                               |Local port name of the INavigation2D interface    |
 * |NAVIGATION_CLIENT |remote navigation  | string |     |                               |Remote port name of the INavigation2D interface   |
 * |NAVIGATION_CLIENT |remote map         | string |     |                               |Remote port name of the IMap2D interface          |
 * |NAVIGATION_CLIENT |remote localizer   | string |     |                               |Remote port name of the ILocalization2D interface |
 * |                  |period             | double |s    |1.0                            |Module period                                     |
 * |                  |name               | string |     |/ros2GoalPoseRedirector        |Module name                                       |
 * |                  |node_name          | string |     |yarp_ros2_goal_pose_redirector |ROS2 node name                                    |
 * |                  |topic_name         | string |     |/goal_pose                     |ROS2 topic name                                   |
 *
 */

class Ros2GoalPoseRedirector : public yarp::os::RFModule
{
protected:
    double                   m_period;
    std::string              m_name;
    std::string              m_nodeName;
    std::string              m_topicName;
    rclcpp::Node::SharedPtr  m_node;
    yarp::dev::PolyDriver            m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D *m_iNav2D{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goalPoseSub;

public:
    Ros2GoalPoseRedirector();
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

#endif // ROS2GOALPOSEREDIRECTOR_H