/* 
* Copyright(C)2018 ICub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email : marco.randazzo@iit.it
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#define _USE_MATH_DEFINES

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include <cmath>
#include <yarp/math/Math.h>
#include "rosNavigator.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef DEG2RAD
#define DEG2RAD M_PI / 180.0
#endif

YARP_LOG_COMPONENT(ROS_NAV, "navigation.rosNavigator")

rosNavigator::rosNavigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_rosNodeName = "/rosNavigator";
    m_rosTopicName_goal = "/move_base/goal";
    m_rosTopicName_cancel = "/move_base/cancel";
    m_rosTopicName_simple_goal = "/move_base_simple/goal";
    m_rosTopicName_feedback = "/move_base/feedback";
    m_rosTopicName_status = "/move_base/status";
    m_rosTopicName_result = "/move_base/result";
    m_navigation_status = navigation_status_idle;
    m_nameof_remote_localization_port = LOCALIZATION_REMOTE_PORT_DEFAULT;
    m_rosTopicName_globalOccupancyGrid = "/move_base/global_costmap/costmap";
    m_rosTopicName_localOccupancyGrid = "/move_base/local_costmap/costmap";
    m_rosTopicName_recoveryStatus = "/move_base/recovery_status";
    m_rosTopicName_globalPath = "/move_base/NavfnROS/plan";
    m_rosTopicName_localPath = " /move_base/DWAPlannerROS/local_plan";
    m_abs_frame_id = "map";
    m_moveBase_isAction = true;
    m_last_goal_id = "goal_0";
}

bool rosNavigator::open(yarp::os::Searchable &config)
{
    Bottle &rosGroup = config.findGroup("ROS");
    if (rosGroup.isNull())
    {
        //do nothing
        yCInfo(ROS_NAV) << "rosNavigator: 'ROS' group param not found. Using defaults.";
    }
    else
    {
        // check for ROS_nodeName parameter
        if (!rosGroup.check("ROS_nodeName"))
        {
            yCError(ROS_NAV) << "rosNavigator: cannot find ROS_nodeName parameter, mandatory when using ROS message";
            return false;
        }

        m_rosNodeName = rosGroup.find("ROS_nodeName").asString(); // TODO: check name is correct
        yCInfo(ROS_NAV) << "rosNavigator: rosNodeName is " << m_rosNodeName;

        // check for ROS_topicName parameter
        if (!rosGroup.check("ROS_topicName_goal"))
        {
            yCError(ROS_NAV) << " rosNavigator: cannot find ROS_topicName parameter, mandatory when using ROS message";
            return false;
        }
      
        m_rosTopicName_goal = rosGroup.find("ROS_topicName_goal").asString();
        yCInfo(ROS_NAV) << "rosNavigator: ROS_topicName_goal is " << m_rosTopicName_goal;

        // check for ROS_goalIsAction parameter
        if (!rosGroup.check("ROS_goalIsAction"))
        {
            yError() << " rosNavigator: cannot find ROS_goalIsAction parameter, mandatory when using ROS message";
            return false;
        }
        m_moveBase_isAction = rosGroup.find("ROS_goalIsAction").asBool();
        yCInfo(ROS_NAV) << "rosNavigator: ROS_goalIsAction is " << m_moveBase_isAction;
    }

    //open ROS stuff
    m_rosNode = new yarp::os::Node(m_rosNodeName);
    if (m_rosNode == nullptr)
    {
        yCError(ROS_NAV) << " opening " << m_rosNodeName << " Node, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_goal.topic(m_rosTopicName_goal))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_cancel.topic(m_rosTopicName_cancel))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_cancel << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_simple_goal.topic(m_rosTopicName_simple_goal))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_simple_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_feedback.topic(m_rosTopicName_feedback))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_feedback << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_status.topic(m_rosTopicName_status))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_status << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    m_rosNavigationStatusCallback = new RosNavigationStatusCallback(this);
    m_rosSubscriber_status.useCallback(*m_rosNavigationStatusCallback);
    if (!m_rosSubscriber_result.topic(m_rosTopicName_result))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_result << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_globalOccupancyGrid.topic(m_rosTopicName_globalOccupancyGrid))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_globalOccupancyGrid << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_localOccupancyGrid.topic(m_rosTopicName_localOccupancyGrid))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_localOccupancyGrid << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_recoveryStatus.topic(m_rosTopicName_recoveryStatus))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_recoveryStatus << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    m_rosRecoveryStatusCallback = new RosRecoveryStatusCallback(this);
    m_rosSubscriber_recoveryStatus.useCallback(*m_rosRecoveryStatusCallback);
    m_rosSubscriber_recoveryStatus.setStrict();
    if (!m_rosSubscriber_globalPath.topic(m_rosTopicName_globalPath))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_globalPath << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_localPath.topic(m_rosTopicName_localPath))
    {
        yCError(ROS_NAV) << " opening " << m_rosTopicName_localPath << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }

    bool bth = this->start();

    return bth;
}

bool rosNavigator::close()
{
    if (m_rosNode != nullptr)
    {
        m_rosNode->interrupt();
        delete m_rosNode;
        m_rosNode = nullptr;
    }
    m_rosSubscriber_status.close();
    delete m_rosNavigationStatusCallback;
    m_rosSubscriber_recoveryStatus.close();
    delete m_rosRecoveryStatusCallback;
    return true;
}

bool rosNavigator::threadInit()
{
    //localization
    Property loc_options;
    loc_options.put("device", LOCALIZATION_CLIENT_DEVICE_DEFAULT);
    loc_options.put("local", m_name + "/localizationClient");
    loc_options.put("remote", m_nameof_remote_localization_port);
    if (m_pLoc.open(loc_options) == false)
    {
        yCError(ROS_NAV) << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_pLoc.isValid() == false || m_iLoc == 0)
    {
        yCError(ROS_NAV) << "Unable to view localization interface";
        return false;
    }

    //map_server
    Property map_options;
    map_options.put("device", MAP_CLIENT_DEVICE_DEFAULT);
    map_options.put("local", "/robotPathPlanner"); //This is just a prefix. map2DClient will complete the port name.
    map_options.put("remote", m_nameof_remote_map_port);
    if (m_pMap.open(map_options) == false)
    {
        yCError(ROS_NAV) << "Unable to open mapClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yCError(ROS_NAV) << "Unable to open map interface";
        return false;
    }

    //read localization data and get the map
    readLocalizationData();

    m_stats_time_curr = yarp::os::Time::now();
    m_stats_time_last = yarp::os::Time::now();

    return true;
}

void rosNavigator::threadRelease()
{
    m_pLoc.close();
}

bool rosNavigator::readLocalizationData()
{
    //gets the current position of the robot from the localization server
    //and reloads the map from the map server if it the map indicated in the current position of the robot
    //is different from the one currently used
    bool ret = m_iLoc->getCurrentPosition(m_current_position);
    if (ret)
    {
        //reset watchdog
    }
    else
    {
        yCError(ROS_NAV) << "Unable to receive localization data";
        return false;
    }

    if (m_current_position.map_id != m_global_map.getMapName())
    {
        yCWarning(ROS_NAV) << "Current map name (" << m_global_map.getMapName() << ") != m_localization_data.map_id (" << m_current_position.map_id << ")";
        yCInfo(ROS_NAV) << "Asking the map '" << m_current_position.map_id << "' to the MAP server";
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


bool rosNavigator::reloadCurrentMap()
{
    yCDebug(ROS_NAV, "Reloading map %s from server", m_global_map.m_map_name.c_str());
    bool map_get_succesfull = this->m_iMap->get_map(m_current_position.map_id, m_global_map);
    if (map_get_succesfull)
    {
        yCInfo(ROS_NAV) << "Map '" << m_current_position.map_id << "' successfully obtained from server";
        m_global_map.crop(-1, -1, -1, -1);
        //m_global_map.enlargeObstacles(m_robot_radius);
        //yCDebug(ROS_NAV) << "Obstacles enlargement performed (" << m_robot_radius << "m)";
        return true;
    }
    else
    {
        yCError(ROS_NAV) << "Unable to get map '" << m_current_position.map_id << "' from map server";
        std::vector<std::string> names_vector;
        m_iMap->get_map_names(names_vector);
        std::string names = "Known maps are:";
        for (auto it = names_vector.begin(); it != names_vector.end(); it++)
        {
            names = names + " " + (*it);
        }
        yCInfo(ROS_NAV) << names;
        return false;
    }
    return true;
}

void rosNavigator::run()
{
    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        m_stats_time_last = m_stats_time_curr;
        bool err = false;
        /*if (m_laser_timeout_counter > TIMEOUT_MAX)
        {
            yCError("timeout, no laser data received!\n");
            err = true;
        }
        if (m_loc_timeout_counter > TIMEOUT_MAX)
        {
            yCError(" timeout, no localization data received!\n");
            err = true;
        }
        if (m_inner_status_timeout_counter > TIMEOUT_MAX)
        {
            yCError("timeout, no status info received!\n");
            err = true;
        }*/
        if (err == false)
        {
            NavigationStatusEnum status;
            getNavigationStatus(status);
            yCInfo(ROS_NAV) << "rosNavigator running, ALL ok. Navigation status:" << getStatusAsString(status);
        }
    }

    bool b1 = readLocalizationData();

    if (0)
    {
        yarp::rosmsg::nav_msgs::OccupancyGrid *ros_global_map = m_rosSubscriber_globalOccupancyGrid.read(false);
        if (global_map)
        {
            m_global_map.setSize_in_cells(ros_global_map->info.width, ros_global_map->info.height);
            m_global_map.setResolution(ros_global_map->info.resolution);
            m_global_map.setMapName("global_map");
            yarp::math::Quaternion quat(ros_global_map->info.origin.orientation.x,
                                        ros_global_map->info.origin.orientation.y,
                                        ros_global_map->info.origin.orientation.z,
                                        ros_global_map->info.origin.orientation.w);
            yarp::sig::Matrix mat = quat.toRotationMatrix4x4();
            yarp::sig::Vector vec = yarp::math::dcm2rpy(mat);
            double orig_angle = vec[2];
            m_global_map.setOrigin(ros_global_map->info.origin.position.x, ros_global_map->info.origin.position.y, orig_angle);
            for (size_t y = 0; y < ros_global_map->info.height; y++)
            {
                for (size_t x = 0; x < ros_global_map->info.width; x++)
                {
                    XYCell cell(x, ros_global_map->info.height - 1 - y);
                    double occ = ros_global_map->data[x + y * ros_global_map->info.width];
                    m_global_map.setOccupancyData(cell, occ);

                    if (occ >= 0 && occ <= 70)
                        m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_FREE);
                    else if (occ >= 71 && occ <= 100)
                        m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_WALL);
                    else
                        m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_UNKNOWN);
                }
            }
        }
    }

    if (0)
    {
        yarp::rosmsg::nav_msgs::OccupancyGrid *ros_local_map = m_rosSubscriber_localOccupancyGrid.read(false);
        if (local_map)
        {
            m_local_map.setSize_in_cells(ros_local_map->info.width, ros_local_map->info.height);
            m_local_map.setResolution(ros_local_map->info.resolution);
            m_local_map.setMapName("local_map");
            yarp::math::Quaternion quat(ros_local_map->info.origin.orientation.x,
                                        ros_local_map->info.origin.orientation.y,
                                        ros_local_map->info.origin.orientation.z,
                                        ros_local_map->info.origin.orientation.w);
            yarp::sig::Matrix mat = quat.toRotationMatrix4x4();
            yarp::sig::Vector vec = yarp::math::dcm2rpy(mat);
            double orig_angle = vec[2];
            m_local_map.setOrigin(ros_local_map->info.origin.position.x, ros_local_map->info.origin.position.y, orig_angle);
            for (size_t y = 0; y < ros_local_map->info.height; y++)
            {
                for (size_t x = 0; x < ros_local_map->info.width; x++)
                {
                    XYCell cell(x, ros_local_map->info.height - 1 - y);
                    double occ = ros_local_map->data[x + y * ros_local_map->info.width];
                    m_local_map.setOccupancyData(cell, occ);

                    if (occ >= 0 && occ <= 70)
                        m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_FREE);
                    else if (occ >= 71 && occ <= 100)
                        m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_WALL);
                    else
                        m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_UNKNOWN);
                }
            }
        }
    }
}

bool rosNavigator::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    // if (m_navigation_status == navigation_status_idle || m_navigation_status == navigation_status_goal_reached || m_navigation_status == navigation_status_aborted)
    // {
    yarp::rosmsg::geometry_msgs::Pose gpose;
    gpose.position.x = loc.x;
    gpose.position.y = loc.y;
    gpose.position.z = 0;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0] = 0;
    v[1] = 0;
    v[2] = 1;
    v[3] = loc.theta * DEG2RAD;
    q.fromAxisAngle(v);
    gpose.orientation.x = q.x();
    gpose.orientation.y = q.y();
    gpose.orientation.z = q.z();
    gpose.orientation.w = q.w();
    m_current_goal = loc;

    double temp_current_time_secs = yarp::os::Time::now();
    m_last_goal_id = "goal_" + std::to_string(temp_current_time_secs);

    if (m_moveBase_isAction)
    {
        yarp::rosmsg::move_base_msgs::MoveBaseActionGoal &goal = m_rosPublisher_goal.prepare();
        goal.clear();
        goal.header.frame_id = "";
        goal.header.seq = 0;
        goal.header.stamp.sec = temp_current_time_secs;
        goal.header.stamp.nsec = 0;
        goal.goal_id.id = m_last_goal_id;
        goal.goal.target_pose.header.frame_id = m_abs_frame_id;
        goal.goal.target_pose.header.seq = 0;
        goal.goal.target_pose.header.stamp.sec = temp_current_time_secs;
        goal.goal.target_pose.header.stamp.nsec = 0;
        goal.goal.target_pose.pose = gpose;
        m_rosPublisher_goal.write();
    }
    else
    {
        yarp::rosmsg::geometry_msgs::PoseStamped &pos = m_rosPublisher_simple_goal.prepare();
        pos.clear();
        pos.header.frame_id = m_abs_frame_id;
        pos.header.seq = 0;
        pos.header.stamp.sec = temp_current_time_secs;
        pos.header.stamp.nsec = 0;
        pos.pose = gpose;
        m_rosPublisher_simple_goal.write();
    }
    return true;
    // }
    // yCError(ROS_NAV) << "A navigation task is already running. Stop it first";
    // return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    NavigationStatusEnum status;
    getNavigationStatus(status);
    if (status == navigation_status_idle)
    {
        Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x;             //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y;             //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - theta; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yCError(ROS_NAV) << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y)
{
    NavigationStatusEnum status;
    getNavigationStatus(status);
    if (status == navigation_status_idle)
    {
        Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x;         //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y;         //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - 0; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yCError(ROS_NAV) << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::getNavigationStatus(NavigationStatusEnum &status)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    status = m_navigation_status;
    return true;
}

void rosNavigator::setNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum status){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_navigation_status = status;
}

void rosNavigator::setIsRecovering(bool isRecovering){
    std::lock_guard<std::mutex> lock(m_mutex);
    m_isRecovering = isRecovering;
}

bool rosNavigator::getIsRecovering(){
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_isRecovering;
}

bool rosNavigator::stopNavigation()
{
    yarp::rosmsg::actionlib_msgs::GoalID &goal_id = m_rosPublisher_cancel.prepare();
    goal_id.clear();
    goal_id.id = m_last_goal_id;
    m_rosPublisher_cancel.write();

    setNavigationStatus(navigation_status_idle);
    return true;
}

bool rosNavigator::getAbsoluteLocationOfCurrentTarget(Map2DLocation &target)
{
    target = m_current_goal;
    return true;
}

bool rosNavigator::getRelativeLocationOfCurrentTarget(double &x, double &y, double &theta)
{
    x = m_current_goal.x - m_current_position.x;             // @@@THIS NEEDS TO BE FIXED
    y = m_current_goal.y - m_current_position.y;             // @@@THIS NEEDS TO BE FIXED
    theta = m_current_goal.theta - m_current_position.theta; // @@@THIS NEEDS TO BE FIXED
    return true;
}

bool rosNavigator::suspendNavigation(double time)
{
    yCError(ROS_NAV) << "Unable to pause current navigation task";
    return false;
}

bool rosNavigator::resumeNavigation()
{
    yCError(ROS_NAV) << "Unable to resume any paused navigation task";
    return false;
}

bool rosNavigator::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath &waypoints)
{
    if (trajectory_type==global_trajectory)
    {
        yarp::rosmsg::nav_msgs::Path *globalPath = m_rosSubscriber_globalPath.read(false);
        if (globalPath && globalPath->poses.size() != 0 )
        {
			m_global_plan.clear();
            for(std::vector<yarp::rosmsg::geometry_msgs::PoseStamped>::const_iterator it = globalPath->poses.begin(); it!= globalPath->poses.end();  ++it)
            {
                Map2DLocation loc;
                loc.map_id = "";
                loc.x = it->pose.position.x;
                loc.y = it->pose.position.y;
                loc.theta = it->pose.orientation.w;

                m_global_plan.push_back(loc);
            }

        }
    	waypoints = m_global_plan;
    }
    if (trajectory_type==local_trajectory)
    {
        yarp::rosmsg::nav_msgs::Path *localPath = m_rosSubscriber_localPath.read(false);
        if (localPath && localPath->poses.size() != 0 )
        {
			m_local_plan.clear();
            for(std::vector<yarp::rosmsg::geometry_msgs::PoseStamped>::const_iterator it = localPath->poses.begin(); it!= localPath->poses.end();  ++it)
            {
                Map2DLocation loc;
                loc.map_id = "";
                loc.x = it->pose.position.x;
                loc.y = it->pose.position.y;
                loc.theta = it->pose.orientation.w;

                m_local_plan.push_back(loc);
            }

        }
    	waypoints = m_local_plan;
    }
    return true;
}

/**
* Returns the current waypoint pursued by the navigation algorithm
* @param curr_waypoint the current waypoint pursued by the navigation algorithm
* @return true/false
*/
bool rosNavigator::getCurrentNavigationWaypoint(Map2DLocation &curr_waypoint)
{
    // @@@@ TEMPORARY to stop spam!

    //yCDebug(ROS_NAV) << "Not yet implemented";
    //return false;
    return true;
}

bool rosNavigator::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D &map)
{
    if (map_type == NavigationMapTypeEnum::global_map)
    {
        map = m_global_map;
        return true;
    }
    else if (map_type == NavigationMapTypeEnum::local_map)
    {
        map = m_local_map;
        return true;
    }
    yCError(ROS_NAV) << "rosNavigator::getCurrentNavigationMap invalid type";
    return false;
}

bool rosNavigator::recomputeCurrentNavigationPath()
{
    NavigationStatusEnum status;
    getNavigationStatus(status);
    if (status == navigation_status_moving)
    {
        yCDebug(ROS_NAV) << "Not yet implemented";
        return false;
    }
    yCError(ROS_NAV) << "Unable to recompute path. Navigation task not assigned yet.";
    return false;
}

std::string rosNavigator::getStatusAsString(NavigationStatusEnum status)
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

bool rosNavigator::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    yCDebug(ROS_NAV) << "Not yet implemented";
    return false;
}

bool rosNavigator::getLastVelocityCommand(double &x_vel, double &y_vel, double &theta_vel)
{
    yCDebug(ROS_NAV) << "Not yet implemented";
    return false;
}

RosNavigationStatusCallback::RosNavigationStatusCallback(rosNavigator *nav) : m_nav(nav)
{}

void RosNavigationStatusCallback::onRead(yarp::rosmsg::actionlib_msgs::GoalStatusArray &statusArray)
{
    if (!m_nav->getIsRecovering())
    {
        if (statusArray.status_list.size() != 0)
        {
            // Comments for the goal status types are taken from http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
            switch (statusArray.status_list[statusArray.status_list.size() - 1].status)
            {
            case yarp::rosmsg::actionlib_msgs::GoalStatus::PENDING:     // The goal has yet to be processed by the action server
            {
                m_nav->setNavigationStatus(navigation_status_preparing_before_move);
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::ACTIVE:      // The goal is currently being processed by the action server
            {
                m_nav->setNavigationStatus(navigation_status_moving);
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::PREEMPTED:   // The goal received a cancel request after it started executing
                                                                        // and has since completed its execution (Terminal State)
            {
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::SUCCEEDED:   // The goal was achieved successfully by the action server (Terminal State)
            {
                m_nav->setNavigationStatus(navigation_status_goal_reached);
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::ABORTED:     // The goal was aborted during execution by the action server due
                                                                        // to some failure (Terminal State)
            {
                m_nav->setNavigationStatus(navigation_status_aborted);
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::REJECTED:    // The goal was rejected by the action server without being processed,
                                                                        // because the goal was unattainable or invalid (Terminal State)
            {
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::PREEMPTING:  // The goal received a cancel request after it started executing
                                                                        // and has not yet completed execution
            {
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::RECALLING:   // The goal received a cancel request before it started executing,
                                                                        // but the action server has not yet confirmed that the goal is canceled
            {
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::RECALLED:    // The goal received a cancel request before it started executing
                                                                        // and was successfully cancelled (Terminal State)
            {
            }
            break;
            case yarp::rosmsg::actionlib_msgs::GoalStatus::LOST:        // An action client can determine that a goal is LOST. This should not be
                                                                        // sent over the wire by an action server
            {
            }
            break;
            default:
            {
            }
            break;
            }
        }
    }
}

RosRecoveryStatusCallback::RosRecoveryStatusCallback(rosNavigator *nav) : m_nav(nav)
{
}

void RosRecoveryStatusCallback::onRead(yarp::rosmsg::move_base_msgs::RecoveryStatus &recoveryInfo)
{
    if (recoveryInfo.recovery_behavior_name.length() > 0)
    {        
        if (recoveryInfo.total_number_of_recoveries == 999){
			yCInfo(ROS_NAV) << "Navigation finished recovery:" << recoveryInfo.recovery_behavior_name;
			m_nav->setNavigationStatus(navigation_status_waiting_obstacle);
			m_nav->setIsRecovering(false);
		}else{
			yCInfo(ROS_NAV) << "Navigation status set to recovery:" << recoveryInfo.recovery_behavior_name;
			m_nav->setNavigationStatus(navigation_status_thinking);
			m_nav->setIsRecovering(true);
		}
    }
}
