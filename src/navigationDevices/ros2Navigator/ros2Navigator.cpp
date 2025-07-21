/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ros2Navigator.h"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/version.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std::placeholders;

#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0
#endif

YARP_LOG_COMPONENT(ROS2_NAV, "navigation.ros2Navigator")

ros2Navigator::ros2Navigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_nodeName = "ros2Navigator";
    m_name = "ros2Navigator";
    m_navigation_status = navigation_status_idle;
    m_nameof_remote_localization_port = LOCALIZATION_REMOTE_PORT_DEFAULT;
    // m_rosTopicName_globalOccupancyGrid = "/move_base/global_costmap/costmap";
    // m_rosTopicName_localOccupancyGrid = "/move_base/local_costmap/costmap";
    m_rosTopicName_globalPath = "/plan";
    m_rosTopicName_localPath = "/local_plan";
    m_abs_frame_id = "map";
}

bool ros2Navigator::open(yarp::os::Searchable &config)
{

    yCDebug(ROS2_NAV) << "ros2Localizer configuration: \n"
                      << config.toString().c_str();

    Bottle general_group = config.findGroup("ROS2NAVIGATOR_GENERAL");
    if (!general_group.isNull())
    {
        if (general_group.check("name"))
        {
            m_name = general_group.find("name").asString();
        }
    }

    // localization
    Property loc_options;
    loc_options.put("device", LOCALIZATION_CLIENT_DEVICE_DEFAULT);
    loc_options.put("local", "/" + m_name + "/localizationClient");
    loc_options.put("remote", m_nameof_remote_localization_port);
    if (m_pLoc.open(loc_options) == false)
    {
        yCError(ROS2_NAV) << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_pLoc.isValid() == false || m_iLoc == 0)
    {
        yCError(ROS2_NAV) << "Unable to view localization interface";
        return false;
    }

    // map_server
    Property map_options;
    map_options.put("device", MAP_CLIENT_DEVICE_DEFAULT);
    map_options.put("local", "/robotPathPlanner"); // This is just a prefix. map2DClient will complete the port name.
    map_options.put("remote", m_nameof_remote_map_port);
    if (m_pMap.open(map_options) == false)
    {
        yCError(ROS2_NAV) << "Unable to open mapClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yCError(ROS2_NAV) << "Unable to open map interface";
        return false;
    }

    Bottle ros_group = config.findGroup("ROS2");
    if (ros_group.isNull())
    {
        yCInfo(ROS2_NAV) << "ROS2 group param not found. Using defaults.";
    }
    else
    {
        if (!ros_group.check("node_name"))
        {
            yCWarning(ROS2_NAV) << "No node_name specified. Using device name instead";
            m_nodeName = m_name;
        }
        else
        {
            m_nodeName = ros_group.find("node_name").asString();
        }
    }

    if (!m_node)
    {
        m_node = NodeCreator::createNode(m_nodeName);
        client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(m_node, "navigate_to_pose");
        nav_through_pose_client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(m_node, "navigate_through_poses");
        m_ros2Subscriber_globalPath = m_node->create_subscription<nav_msgs::msg::Path>(m_rosTopicName_globalPath, 10, std::bind(&ros2Navigator::globalPath_callback, this, _1));
        m_ros2Subscriber_localPath = m_node->create_subscription<nav_msgs::msg::Path>(m_rosTopicName_localPath, 10, std::bind(&ros2Navigator::localPath_callback, this, _1));

        // Navigation feedback lambda
        navigation_feedback_sub_ = m_node->create_subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>(
            "navigate_to_pose/_action/feedback", rclcpp::SystemDefaultsQoS(),
            [this](const nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage::SharedPtr msg)
            {
                // geometry_msgs/PoseStamped current_pose
                // builtin_interfaces/Duration navigation_time
                // builtin_interfaces/Duration estimated_time_remaining
                // int16 number_of_recoveries
                // float32 distance_remaining
                yCInfoThrottle(ROS2_NAV, 1) << "Navigation ETA:" << msg->feedback.estimated_time_remaining.sec << "with" << msg->feedback.distance_remaining << "meters remaining.";
            });
        navigation_goal_status_sub_ = m_node->create_subscription<action_msgs::msg::GoalStatusArray>(
            "navigate_to_pose/_action/status", rclcpp::SystemDefaultsQoS(),
            [this](const action_msgs::msg::GoalStatusArray::SharedPtr msg)
            {
                switch (msg->status_list.back().status)
                {
                case action_msgs::msg::GoalStatus::STATUS_ACCEPTED: // The goal has been accepted and is awaiting execution.
                    setNavigationStatus(navigation_status_preparing_before_move);
                    yCDebug(ROS2_NAV) << "Goal was successfully accepted by the action server.";
                    break;
                case action_msgs::msg::GoalStatus::STATUS_EXECUTING: // The goal is currently being executed by the action server.
                    setNavigationStatus(navigation_status_moving);
                    break;
                case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED: // The goal was achieved successfully by the action server.
                    setNavigationStatus(navigation_status_goal_reached);
                    break;
                case action_msgs::msg::GoalStatus::STATUS_ABORTED: // The goal was terminated by the action server without an external request.
                    setNavigationStatus(navigation_status_aborted);
                    break;
                case action_msgs::msg::GoalStatus::STATUS_CANCELING: // The client has requested that the goal be canceled and the action server has accepted the cancel request.
                case action_msgs::msg::GoalStatus::STATUS_CANCELED: // The goal was canceled after an external request from an action client.
                    setNavigationStatus(navigation_status_idle);
                    break;
                default: // Indicates status has not been properly set.
                    setNavigationStatus(navigation_status_error);
                    yCError(ROS2_NAV) << "Navigation status: STATUS_UNKNOWN";
                    return;
                }
            });
    }

    bool b = this->start();
    return b;
}
void ros2Navigator::globalPath_callback(const nav_msgs::msg::Path &msg)
{
    if (msg.poses.size() != 0)
    {
        m_global_plan.clear();
        for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it = msg.poses.begin(); it != msg.poses.end(); ++it)
        {
            Map2DLocation loc;
            loc.map_id = "";
            loc.x = it->pose.position.x;
            loc.y = it->pose.position.y;
            loc.theta = it->pose.orientation.w;

            m_global_plan.push_back(loc);
        }
    }
}

void ros2Navigator::localPath_callback(const nav_msgs::msg::Path &msg)
{
    if (msg.poses.size() != 0)
    {
        m_local_plan.clear();
        for (std::vector<geometry_msgs::msg::PoseStamped>::const_iterator it = msg.poses.begin(); it != msg.poses.end(); ++it)
        {
            Map2DLocation loc;
            loc.map_id = "";
            loc.x = it->pose.position.x;
            loc.y = it->pose.position.y;
            loc.theta = it->pose.orientation.w;

            m_local_plan.push_back(loc);
        }
    }
}

bool ros2Navigator::close()
{
    m_pLoc.close();
    m_pMap.close();
    delete m_iMap;
    m_iMap = nullptr;
    delete m_iLoc;
    m_iLoc = nullptr;
    return true;
}

bool ros2Navigator::threadInit()
{
    // read localization data and get the map
    readLocalizationData();
    yCInfo(ROS2_NAV) << "Thread initialized";
    return true;
}

void ros2Navigator::threadRelease()
{
    if (m_spun)
    {
        if (m_innerSpinner->isRunning())
        {
            m_innerSpinner->stop();
        }
        delete m_innerSpinner;
        m_innerSpinner = nullptr;
        m_spun = false;
    }
}

bool ros2Navigator::readLocalizationData()
{
    // gets the current position of the robot from the localization server
    // and reloads the map from the map server if it the map indicated in the current position of the robot
    // is different from the one currently used
    bool ret = m_iLoc->getCurrentPosition(m_current_position);
    if (ret)
    {
        // reset watchdog
    }
    else
    {
        yCError(ROS2_NAV) << "Unable to receive localization data";
        return false;
    }

    if (m_current_position.map_id != m_global_map.getMapName())
    {
        yCWarning(ROS2_NAV) << "Current map name (" << m_global_map.getMapName() << ") != m_localization_data.map_id (" << m_current_position.map_id << ")";
        yCInfo(ROS2_NAV) << "Asking the map '" << m_current_position.map_id << "' to the MAP server";
        bool b = reloadCurrentMap();

        yarp::os::Time::delay(1.0);
        if (b)
        {
            return true;
        }
        else
        {
            return true; //@@@consider changing this to false
        }
    }
    return true;
}

bool ros2Navigator::reloadCurrentMap()
{
    yCDebug(ROS2_NAV, "Reloading map %s from server", m_global_map.m_map_name.c_str());
    bool map_get_succesfull = this->m_iMap->get_map(m_current_position.map_id, m_global_map);
    if (map_get_succesfull)
    {
        yCInfo(ROS2_NAV) << "Map '" << m_current_position.map_id << "' successfully obtained from server";
        m_global_map.crop(-1, -1, -1, -1);
        // m_global_map.enlargeObstacles(m_robot_radius);
        // yCDebug(ROS2_NAV) << "Obstacles enlargement performed (" << m_robot_radius << "m)";
        return true;
    }
    else
    {
        yCError(ROS2_NAV) << "Unable to get map '" << m_current_position.map_id << "' from map server";
        std::vector<std::string> names_vector;
        m_iMap->get_map_names(names_vector);
        std::string names = "Known maps are:";
        for (auto it = names_vector.begin(); it != names_vector.end(); it++)
        {
            names = names + " " + (*it);
        }
        yCInfo(ROS2_NAV) << names;
        return false;
    }
    return true;
}

void ros2Navigator::run()
{
    if (!m_spun)
    {
        m_innerSpinner = new Ros2Spinner(m_node);
        m_innerSpinner->start();
        m_spun = true;
    }

    bool b1 = readLocalizationData();
}

yarp::dev::ReturnValue ros2Navigator::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    m_current_goal = loc;
    if (!client_ptr_->wait_for_action_server())
    {
        yCWarning(ROS2_NAV) << "Action server not available after waiting for seconds.";
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = loc.x;
    goal_pose.position.y = loc.y;
    goal_pose.position.z = 0;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0] = 0;
    v[1] = 0;
    v[2] = 1;
    v[3] = loc.theta * DEG2RAD;
    q.fromAxisAngle(v);
    goal_pose.orientation.x = q.x();
    goal_pose.orientation.y = q.y();
    goal_pose.orientation.z = q.z();
    goal_pose.orientation.w = q.w();

    double temp_current_time_secs = yarp::os::Time::now();
    goal_msg.pose.header.frame_id = m_abs_frame_id;
    goal_msg.pose.header.stamp.sec = temp_current_time_secs;
    goal_msg.pose.header.stamp.nanosec = 0;
    goal_msg.pose.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    setNavigationStatus(navigation_status_preparing_before_move);
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    Map2DLocation loc;
    loc.map_id = m_current_position.map_id;
    loc.x = m_current_position.x + x;
    loc.y = m_current_position.y + y;
    loc.theta = m_current_position.theta + theta;
    return gotoTargetByAbsoluteLocation(loc);
}

yarp::dev::ReturnValue ros2Navigator::gotoTargetByRelativeLocation(double x, double y)
{
    Map2DLocation loc;
    loc.map_id = m_current_position.map_id;
    loc.x = m_current_position.x + x;
    loc.y = m_current_position.y + y;
    loc.theta = m_current_position.theta;
    return gotoTargetByAbsoluteLocation(loc);
}

yarp::dev::ReturnValue ros2Navigator::followPath(const yarp::dev::Nav2D::Map2DPath &path)
{

    if (!nav_through_pose_client_ptr_->wait_for_action_server())
    {
        yCWarning(ROS2_NAV) << "Action server not available after waiting for seconds.";
    }

    auto goal_msg = nav2_msgs::action::NavigateThroughPoses::Goal();

    std::vector<geometry_msgs::msg::PoseStamped> poses; // vector of 2Dposes
    for (auto location = path.cbegin(); location != path.cend(); location++){
        geometry_msgs::msg::Pose goal_pose;

        goal_pose.position.x = location->x;
        goal_pose.position.y = location->y;
        goal_pose.position.z = 0;
        yarp::math::Quaternion q;
        yarp::sig::Vector v(4);
        v[0] = 0;
        v[1] = 0;
        v[2] = 1;
        v[3] = location->theta * DEG2RAD;
        q.fromAxisAngle(v);
        goal_pose.orientation.x = q.x();
        goal_pose.orientation.y = q.y();
        goal_pose.orientation.z = q.z();
        goal_pose.orientation.w = q.w();

        geometry_msgs::msg::PoseStamped pose_stamped;
        double temp_current_time_secs = yarp::os::Time::now();
        pose_stamped.header.frame_id = m_abs_frame_id;
        pose_stamped.header.stamp.sec = temp_current_time_secs;
        pose_stamped.header.stamp.nanosec = 0;
        pose_stamped.pose = goal_pose;

        poses.push_back(std::move(pose_stamped)); //we move because it's cheaper than copying
    }

#if RCLCPP_VERSION_GTE(29,3,0)
    goal_msg.poses.goals = poses;
#else
    goal_msg.poses = poses;
#endif

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
    nav_through_pose_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    setNavigationStatus(navigation_status_preparing_before_move);
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::getNavigationStatus(NavigationStatusEnum &status)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    status = m_navigation_status;
    return ReturnValue_ok;
}

void ros2Navigator::setNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum status)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_navigation_status = status;
}

yarp::dev::ReturnValue ros2Navigator::stopNavigation()
{
    client_ptr_->async_cancel_all_goals();
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::getAbsoluteLocationOfCurrentTarget(Map2DLocation &target)
{
    target = m_current_goal;
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::getRelativeLocationOfCurrentTarget(double &x, double &y, double &theta)
{
    x = m_current_goal.x + m_current_position.x;
    y = m_current_goal.y + m_current_position.y;
    theta = m_current_goal.theta + m_current_position.theta;
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::suspendNavigation(double time)
{
    yCError(ROS2_NAV) << "Unable to pause current navigation task";
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::resumeNavigation()
{
    yCError(ROS2_NAV) << "Unable to resume any paused navigation task";
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath &waypoints)
{
    if (trajectory_type == global_trajectory)
    {
        waypoints = m_global_plan;
    }
    if (trajectory_type == local_trajectory)
    {
        waypoints = m_local_plan;
    }
    return ReturnValue_ok;
}

/**
 * Returns the current waypoint pursued by the navigation algorithm
 * @param curr_waypoint the current waypoint pursued by the navigation algorithm
 * @return true/false
 */
yarp::dev::ReturnValue ros2Navigator::getCurrentNavigationWaypoint(Map2DLocation &curr_waypoint)
{
    // @@@@ TEMPORARY to stop spam!

    // yCDebug(ROS2_NAV) << "Not yet implemented";
    // return false;
    return ReturnValue_ok;
}

yarp::dev::ReturnValue ros2Navigator::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D &map)
{
    if (map_type == NavigationMapTypeEnum::global_map)
    {
        map = m_global_map;
        return ReturnValue_ok;
    }
    else if (map_type == NavigationMapTypeEnum::local_map)
    {
        map = m_local_map;
        return ReturnValue_ok;
    }
    yCError(ROS2_NAV) << "rosNavigator::getCurrentNavigationMap invalid type";
    return ReturnValue::return_code::return_value_error_method_failed;
}

yarp::dev::ReturnValue ros2Navigator::recomputeCurrentNavigationPath()
{
    NavigationStatusEnum status;
    getNavigationStatus(status);
    if (status == navigation_status_moving)
    {
        yCDebug(ROS2_NAV) << "Not yet implemented";
        return YARP_METHOD_NOT_YET_IMPLEMENTED();
    }
    yCError(ROS2_NAV) << "Unable to recompute path. Navigation task not assigned yet.";
    return ReturnValue::return_code::return_value_error_method_failed;
}

std::string ros2Navigator::getStatusAsString(NavigationStatusEnum status)
{
    if (status == navigation_status_idle)
        return std::string("navigation_status_idle");
    else if (status == navigation_status_moving)
        return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle)
        return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached)
        return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted)
        return std::string("navigation_status_aborted");
    else if (status == navigation_status_failing)
        return std::string("navigation_status_failing");
    else if (status == navigation_status_paused)
        return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move)
        return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking)
        return std::string("navigation_status_thinking");
    else if (status == navigation_status_error)
        return std::string("navigation_status_error");
    return std::string("navigation_status_error");
}

yarp::dev::ReturnValue ros2Navigator::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    yCDebug(ROS2_NAV) << "Not yet implemented";
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

yarp::dev::ReturnValue ros2Navigator::getLastVelocityCommand(double &x_vel, double &y_vel, double &theta_vel)
{
    yCDebug(ROS2_NAV) << "Not yet implemented";
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}