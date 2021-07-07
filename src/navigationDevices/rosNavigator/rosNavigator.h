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
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/rosmsg/geometry_msgs/PoseStamped.h>
#include <yarp/rosmsg/geometry_msgs/PoseWithCovarianceStamped.h>
#include <yarp/rosmsg/move_base_msgs/MoveBaseGoal.h>
#include <yarp/rosmsg/move_base_msgs/MoveBaseActionGoal.h>
#include <yarp/rosmsg/move_base_msgs/MoveBaseActionResult.h>
#include <yarp/rosmsg/move_base_msgs/MoveBaseActionFeedback.h>
#include <yarp/rosmsg/move_base_msgs/RecoveryStatus.h>
#include <yarp/rosmsg/actionlib_msgs/GoalID.h>
#include <yarp/rosmsg/actionlib_msgs/GoalStatusArray.h>
#include <yarp/rosmsg/nav_msgs/OccupancyGrid.h>
#include <math.h>

#ifndef ROS_NAVIGATOR_H
#define ROS_NAVIGATOR_H

#define DEFAULT_THREAD_PERIOD 0.02 //s

class rosNavigator : public yarp::dev::DeviceDriver,
    public yarp::os::PeriodicThread,
    public yarp::dev::Nav2D::INavigation2DTargetActions,
    public yarp::dev::Nav2D::INavigation2DControlActions
{
protected:
    yarp::dev::PolyDriver              m_pLoc;
    yarp::dev::Nav2D::ILocalization2D* m_iLoc;
    yarp::dev::PolyDriver              m_pMap;
    yarp::dev::Nav2D::IMap2D*          m_iMap;

    yarp::dev::Nav2D::NavigationStatusEnum  m_navigation_status;
    std::string                             m_abs_frame_id;
    std::string                             m_name;
    std::string                             m_remote_localization;
    yarp::dev::Nav2D::Map2DLocation         m_current_position;
    yarp::dev::Nav2D::Map2DLocation         m_current_goal;

    double                            m_stats_time_curr;
    double                            m_stats_time_last;
    double                            m_time_recovery;
    double                            m_time_limit_recovery = 5.0; 

    bool                              m_moveBase_isAction; // False by default
    
    std::string                       m_rosNodeName;
    yarp::os::Node                    *m_rosNode;                  // add a ROS node
    yarp::os::NetUint32               m_rosMsgCounter;             // incremental counter in the ROS message
    yarp::dev::Nav2D::MapGrid2D       m_local_map;
    yarp::dev::Nav2D::MapGrid2D       m_global_map;

    std::string                       m_rosTopicName_goal;
    std::string                       m_rosTopicName_cancel;
    std::string                       m_rosTopicName_simple_goal;
    std::string                       m_rosTopicName_feedback;
    std::string                       m_rosTopicName_status;
    std::string                       m_rosTopicName_result;
    std::string                       m_rosTopicName_globalOccupancyGrid;
    std::string                       m_rosTopicName_localOccupancyGrid;
    std::string                       m_rosTopicName_recoveryStatus;
    std::string                       m_last_goal_id;

    yarp::os::Publisher<yarp::rosmsg::move_base_msgs::MoveBaseActionGoal> m_rosPublisher_goal;
    yarp::os::Publisher<yarp::rosmsg::actionlib_msgs::GoalID> m_rosPublisher_cancel;
    yarp::os::Publisher<yarp::rosmsg::geometry_msgs::PoseStamped> m_rosPublisher_simple_goal;
    yarp::os::Subscriber<yarp::rosmsg::move_base_msgs::MoveBaseActionFeedback> m_rosSubscriber_feedback;
    yarp::os::Subscriber<yarp::rosmsg::actionlib_msgs::GoalStatusArray> m_rosSubscriber_status;
    yarp::os::Subscriber<yarp::rosmsg::move_base_msgs::MoveBaseActionResult> m_rosSubscriber_result;
    yarp::os::Subscriber<yarp::rosmsg::nav_msgs::OccupancyGrid> m_rosSubscriber_localOccupancyGrid;
    yarp::os::Subscriber<yarp::rosmsg::nav_msgs::OccupancyGrid> m_rosSubscriber_globalOccupancyGrid;
    yarp::os::Subscriber<yarp::rosmsg::move_base_msgs::RecoveryStatus> m_rosSubscriber_recoveryStatus;

public:
    rosNavigator();
    ~rosNavigator() {}

public:
    virtual bool open(yarp::os::Searchable& config) override;
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
    * Apply a velocity command. velocities are expressed in the robot reference frame
    * @param x [m/s]
    * @param y [m/s]
    * @param theta [deg/s]
    * @param timeout The velocity command expires after the specified amount of time (by default 0.1 seconds)
    * @return true/false
    */
    virtual bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;

    /**
    * //Gets the last target set through a setNewAbsTarget() command.
    * @return a Map2DLocation containing data of the current target.
    * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
    */
    bool getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target) override;

    /**
    * //Gets the last target set through a setNewRelTarget command, expressed in absolute coordinates.
    * @param a Map2DLocation containing data of the current target.
    * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
    */
    bool getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta) override;

    /**
    * //Gets the status of the current navigation task. Typically stored into navigation_status variable.
    * @return the current navigation status expressed as NavigationStatusEnum.
    */
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status) override;

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
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints) override;

    /**
    * Returns the current waypoint pursued by the navigation algorithm
    * @param curr_waypoint the current waypoint pursued by the navigation algorithm
    * @return true/false
    */
    bool getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint) override;

    /**
    * Returns the current navigation map processed by the navigation algorithm
    * @param map_type the map to be requested (e.g. global, local, etc.)
    * @param map the map, currently used by the navigation algorithm
    * @return true/false
    */
    bool getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map) override;

    /**
    * Forces the navigation system to recompute the path from the current robot position to the current goal.
    * If no goal has been set, the command has no effect.
    * @return true/false
    */
    bool recomputeCurrentNavigationPath() override;
};

#endif
