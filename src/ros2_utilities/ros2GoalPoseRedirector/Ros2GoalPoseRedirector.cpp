/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "Ros2GoalPoseRedirector.h"

#ifndef RAD2DEG
#define RAD2DEG 180.0 / M_PI
#endif

YARP_LOG_COMPONENT(ROS2_GOAL_POSE_REDIRECTOR, "navigation.ros2GoalPoseRedirector")

Ros2GoalPoseRedirector::Ros2GoalPoseRedirector() :
    m_period(1.0)
{
}

bool Ros2GoalPoseRedirector::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    m_name = "/ros2GoalPoseRedirector";
    if(rf.check("name")){m_name = rf.find("name").asString();}

    m_nodeName = "yarp_ros2_goal_pose_redirector";
    if(rf.check("node_name")){m_nodeName = rf.find("node_name").asString();}

    m_topicName = "/goal_pose";
    if(rf.check("topic_name")){m_topicName = rf.find("topic_name").asString();}

    yarp::os::Network::init();

    m_node = rclcpp::Node::make_shared(m_nodeName);

    if(!m_node)
    {
        yCError(ROS2_GOAL_POSE_REDIRECTOR) << "Failed to create ROS2 node";
        return false;
    }

    if(!rf.check("NAVIGATION_CLIENT"))
    {
        yCError(ROS2_GOAL_POSE_REDIRECTOR) << "Missing nav2d section in configuration file";
        return false;
    }
    if(!m_nav2DPoly.open(rf.findGroup("NAVIGATION_CLIENT")))
    {
        yCError(ROS2_GOAL_POSE_REDIRECTOR) << "Failed to open nav2d polydriver";
        return false;
    }

    if(!m_nav2DPoly.isValid())
    {
        yCError(ROS2_GOAL_POSE_REDIRECTOR) << "Failed to open nav2d polydriver";
        return false;
    }

    m_nav2DPoly.view(m_iNav2D);

    if(!m_iNav2D)
    {
        yCError(ROS2_GOAL_POSE_REDIRECTOR) << "Failed to view INavigation2D interface";
        return false;
    }

    m_goalPoseSub = m_node->create_subscription<geometry_msgs::msg::PoseStamped>(
        m_topicName,
        10,
        std::bind(&Ros2GoalPoseRedirector::goalPoseCallback, this, std::placeholders::_1));

    return true;
}

bool Ros2GoalPoseRedirector::close()
{
    m_nav2DPoly.close();
    m_node.reset();
    yarp::os::Network::fini();
    return true;
}

double Ros2GoalPoseRedirector::getPeriod()
{
    return m_period;
}

bool Ros2GoalPoseRedirector::updateModule()
{
    rclcpp::spin(m_node);
    return true;
}

void Ros2GoalPoseRedirector::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    yCInfo(ROS2_GOAL_POSE_REDIRECTOR) << "Received goal pose: " << msg->pose.position.x << " " << msg->pose.position.y;
    yarp::math::Quaternion q;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;
    q.w() = msg->pose.orientation.w;
    yarp::sig::Vector v = q.toAxisAngle();
    double t = v[3]*v[2];
    yarp::dev::Nav2D::Map2DLocation goal;
    goal.x = msg->pose.position.x;
    goal.y = msg->pose.position.y;
    goal.theta = t * RAD2DEG;
    m_iNav2D->gotoTargetByAbsoluteLocation(goal);
}