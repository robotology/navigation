/*
 * Copyright (C)2018 ICub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IMap2D.h>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yarp/math/Math.h>
#include "Ros2Spinner.h"
#include "Ros2Utils.h"
#include "navigation_defines.h"

#ifndef ROS2_NAVIGATOR_H
#define ROS2_NAVIGATOR_H

#define DEFAULT_THREAD_PERIOD 0.02 // s

class ros2Navigator : public yarp::dev::DeviceDriver,
                      public yarp::os::PeriodicThread,
                      public yarp::dev::Nav2D::INavigation2DTargetActions,
                      public yarp::dev::Nav2D::INavigation2DControlActions,
                      public yarp::dev::Nav2D::INavigation2DVelocityActions
{
protected:
    yarp::dev::PolyDriver m_pLoc;
    yarp::dev::Nav2D::ILocalization2D *m_iLoc;
    yarp::dev::PolyDriver m_pMap;
    yarp::dev::Nav2D::IMap2D *m_iMap;

    std::mutex m_mutex;

    yarp::dev::Nav2D::NavigationStatusEnum m_navigation_status;
    std::string m_abs_frame_id;
    std::string m_name;
    std::string m_nameof_remote_localization_port = LOCALIZATION_REMOTE_PORT_DEFAULT;
    std::string m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    yarp::dev::Nav2D::Map2DLocation m_current_position;
    yarp::dev::Nav2D::Map2DLocation m_current_goal;

    bool m_spun{false};

    std::string m_nodeName;
    yarp::dev::Nav2D::MapGrid2D m_local_map;
    yarp::dev::Nav2D::MapGrid2D m_global_map;
    yarp::dev::Nav2D::Map2DPath m_global_plan;
    yarp::dev::Nav2D::Map2DPath m_local_plan;

    // std::string m_rosTopicName_globalOccupancyGrid;
    // std::string m_rosTopicName_localOccupancyGrid;
    std::string m_rosTopicName_globalPath;
    std::string m_rosTopicName_localPath;

    // ROS2
    Ros2Spinner *m_innerSpinner{nullptr};
    rclcpp::Node::SharedPtr m_node{nullptr};
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_ros2Subscriber_globalPath;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_ros2Subscriber_localPath;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::FeedbackMessage>::SharedPtr navigation_feedback_sub_;
    rclcpp::Subscription<nav2_msgs::action::NavigateToPose::Impl::GoalStatusMessage>::SharedPtr navigation_goal_status_sub_;

public:
    ros2Navigator();
    ~ros2Navigator() {}

public:
    virtual bool open(yarp::os::Searchable &config) override;
    virtual bool close() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

private:
    std::string getStatusAsString(yarp::dev::Nav2D::NavigationStatusEnum status);
    bool readLocalizationData();
    bool reloadCurrentMap();

public:
    /**
     * Sets a new navigation target, expressed in the absolute (map) coordinate frame.
     * @param loc the location to be reached
     * @return true/false if the command is accepted
     */
    bool gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;

    /**
     * //Sets a new relative target, expressed in local (robot) coordinate frame.
     * @param v a three-element vector (x,y,theta) representing the location to be reached
     * @return true/false if the command is accepted
     */
    bool gotoTargetByRelativeLocation(double x, double y, double theta) override;

    /**
     * //Sets a new relative target, expressed in local (robot) coordinate frame.
     * @param v a three-element vector (x,y,theta) representing the location to be reached
     * @return true/false if the command is accepted
     */
    bool gotoTargetByRelativeLocation(double x, double y) override;

    /**
     * //Gets the last target set through a setNewAbsTarget() command.
     * @return a Map2DLocation containing data of the current target.
     * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
     */
    bool getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation &target) override;

    /**
     * //Gets the last target set through a setNewRelTarget command, expressed in absolute coordinates.
     * @param a Map2DLocation containing data of the current target.
     * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
     */
    bool getRelativeLocationOfCurrentTarget(double &x, double &y, double &theta) override;

    /**
     * //Gets the status of the current navigation task. Typically stored into navigation_status variable.
     * @return the current navigation status expressed as NavigationStatusEnum.
     */
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum &status) override;

    /**
     * Setter for the internal member variable that holds the current navigation status
     */
    void setNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum status);

    /**
     * //Stops the current navigation task.
     * @return true/false if the command is executed successfully.
     */
    bool stopNavigation() override;

    /**
     * //Pauses the current navigation task.
     * @return true/false if the command is executed successfully.
     */
    bool suspendNavigation(double time) override;

    /**
     * //Resumes a previously paused navigation task.
     * @return true/false if the command is executed successfully.
     */
    bool resumeNavigation() override;

    /**
     * Returns the list of waypoints generated by the navigation algorithm
     * @param waypoints the list of waypoints generated by the navigation algorithm
     * @return true/false
     */
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath &waypoints) override;

    /**
     * Returns the current waypoint pursued by the navigation algorithm
     * @param curr_waypoint the current waypoint pursued by the navigation algorithm
     * @return true/false
     */
    bool getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation &curr_waypoint) override;

    /**
     * Returns the current navigation map processed by the navigation algorithm
     * @param map_type the map to be requested (e.g. global, local, etc.)
     * @param map the map, currently used by the navigation algorithm
     * @return true/false
     */
    bool getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D &map) override;

    /**
     * Forces the navigation system to recompute the path from the current robot position to the current goal.
     * If no goal has been set, the command has no effect.
     * @return true/false
     */
    bool recomputeCurrentNavigationPath() override;

    // INavigation2DVelocityActions methods
    bool getLastVelocityCommand(double &x_vel, double &y_vel, double &theta_vel) override;
    bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;
    void localPath_callback(const nav_msgs::msg::Path &msg);
    void globalPath_callback(const nav_msgs::msg::Path &msg);
};

#endif
