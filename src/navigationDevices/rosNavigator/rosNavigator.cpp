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
#define DEG2RAD M_PI/180.0
#endif

rosNavigator::rosNavigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_rosNodeName = "/rosNavigator";
    m_rosTopicName_goal = "/move_base/goal";
    m_rosTopicName_cancel = "/move_base/cancel";
    m_rosTopicName_simple_goal = "/move_base_simple/goal";
    m_rosTopicName_feedback = "/move_base/feedback";
    m_rosTopicName_status = "/move_base/status";
    m_rosTopicName_result = "/move_base/result";
    m_navigation_status = yarp::dev::navigation_status_idle;
    m_local_name_prefix = "/rosNavigator";
    m_remote_localization = "/localizationServer";
    m_rosTopicName_globalOccupancyGrid = "/move_base/global_costmap/costmap";
    m_rosTopicName_localOccupancyGrid = "/move_base/local_costmap/costmap";
//    m_abs_frame_id = "/odom";
    m_abs_frame_id = "/map";
}

bool rosNavigator::open(yarp::os::Searchable& config)
{
    Bottle &rosGroup = config.findGroup("ROS");
    if (rosGroup.isNull())
    {
        //do nothing
        yInfo() << "rosNavigator: 'ROS' group param not found. Using defaults.";
    }
    else
    {
        // check for ROS_nodeName parameter
        if (!rosGroup.check("ROS_nodeName"))
        {
            yError() << "rosNavigator: cannot find ROS_nodeName parameter, mandatory when using ROS message";
            return false;
        }
        m_rosNodeName = rosGroup.find("ROS_nodeName").asString();  // TODO: check name is correct
        yInfo() << "rosNavigator: rosNodeName is " << m_rosNodeName;

        // check for ROS_topicName parameter
        if (!rosGroup.check("ROS_topicName_goal"))
        {
            yError() << " rosNavigator: cannot find ROS_topicName parameter, mandatory when using ROS message";
            return false;
        }
        m_rosTopicName_goal = rosGroup.find("ROS_topicName_goal").asString();
        yInfo() << "rosNavigator: ROS_topicName_goal is " << m_rosTopicName_goal;
    }

    //open ROS stuff
    m_rosNode = new yarp::os::Node(m_rosNodeName);
    if (m_rosNode == nullptr)
    {
        yError() << " opening " << m_rosNodeName << " Node, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_goal.topic(m_rosTopicName_goal))
    {
        yError() << " opening " << m_rosTopicName_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_cancel.topic(m_rosTopicName_cancel))
    {
        yError() << " opening " << m_rosTopicName_cancel << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_simple_goal.topic(m_rosTopicName_simple_goal))
    {
        yError() << " opening " << m_rosTopicName_simple_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_feedback.topic(m_rosTopicName_feedback))
    {
        yError() << " opening " << m_rosTopicName_feedback << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_status.topic(m_rosTopicName_status))
    {
        yError() << " opening " << m_rosTopicName_status << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_result.topic(m_rosTopicName_result))
    {
        yError() << " opening " << m_rosTopicName_result << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_globalOccupancyGrid.topic(m_rosTopicName_globalOccupancyGrid))
    {
        yError() << " opening " << m_rosTopicName_globalOccupancyGrid << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosSubscriber_localOccupancyGrid.topic(m_rosTopicName_localOccupancyGrid))
    {
        yError() << " opening " << m_rosTopicName_localOccupancyGrid << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }

    this->start();
    return true;
}

bool rosNavigator::close()
{
    if (m_rosNode != nullptr)
    {
        m_rosNode->interrupt();
        delete m_rosNode;
        m_rosNode = nullptr;
    }
    return true;
}

bool rosNavigator::threadInit()
{
    //localization
    Property loc_options;
    loc_options.put("device", "localization2DClient");
    loc_options.put("local", m_local_name_prefix + "/localizationClient");
    loc_options.put("remote", m_remote_localization);
    if (m_pLoc.open(loc_options) == false)
    {
        yError() << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_pLoc.isValid() == false || m_iLoc == 0)
    {
        yError() << "Unable to view localization interface";
        return false;
    }

    //map_server
    Property map_options;
    map_options.put("device", "map2DClient");
    map_options.put("local", "/robotPathPlanner"); //This is just a prefix. map2DClient will complete the port name.
    map_options.put("remote", "/mapServer");
    if (m_pMap.open(map_options) == false)
    {
        yError() << "Unable to open mapClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yError() << "Unable to open map interface";
        return false;
    }

    //get the map
    yInfo() << "Asking for map 'ros_map'...";
    bool b = m_iMap->get_map("ros_map",m_global_map);
    m_global_map.crop(-1,-1,-1,-1);
    if (b)
    {
        yInfo() << "'ros_map' received";
    }
    else
    {
        yError() << "'ros_map' not found";
    }

    m_stats_time_curr = yarp::os::Time::now();
    m_stats_time_last = yarp::os::Time::now();

    return true;
}

void rosNavigator::threadRelease()
{
    m_pLoc.close();
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
            yError("timeout, no laser data received!\n");
            err = true;
        }
        if (m_loc_timeout_counter > TIMEOUT_MAX)
        {
            yError(" timeout, no localization data received!\n");
            err = true;
        }
        if (m_inner_status_timeout_counter > TIMEOUT_MAX)
        {
            yError("timeout, no status info received!\n");
            err = true;
        }*/
        if (err == false)
            yInfo() << "rosNavigator running, ALL ok. Navigation status:" << getStatusAsString(m_navigation_status);
    }

    bool b1 = m_iLoc->getCurrentPosition(m_current_position);

    if (0)
    {
        yarp::rosmsg::nav_msgs::OccupancyGrid* ros_global_map = m_rosSubscriber_globalOccupancyGrid.read(false);
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

                    if (occ >= 0 && occ <= 70)         m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_FREE);
                    else if (occ >= 71 && occ <= 100)  m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_WALL);
                    else                               m_global_map.setMapFlag(cell, MapGrid2D::MAP_CELL_UNKNOWN);
                }
            }
        }
    }

    if (0)
    {
        yarp::rosmsg::nav_msgs::OccupancyGrid* ros_local_map = m_rosSubscriber_localOccupancyGrid.read(false);
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

                    if (occ >= 0 && occ <= 70)         m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_FREE);
                    else if (occ >= 71 && occ <= 100)  m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_WALL);
                    else                               m_local_map.setMapFlag(cell, MapGrid2D::MAP_CELL_UNKNOWN);
                }
            }
        }
    }
    
    yarp::rosmsg::move_base_msgs::MoveBaseActionFeedback* feedback = m_rosSubscriber_feedback.read(false);
    if (feedback)
    {
        switch (feedback->status.status)
        {
           case feedback->status.PENDING: {} break;
           case feedback->status.ACTIVE: {m_navigation_status = yarp::dev::navigation_status_moving; } break;
           case feedback->status.PREEMPTED: {} break;
           case feedback->status.SUCCEEDED: {m_navigation_status = yarp::dev::navigation_status_goal_reached; } break;
           case feedback->status.ABORTED: {m_navigation_status = yarp::dev::navigation_status_aborted; } break;
           case feedback->status.REJECTED: {} break;
           case feedback->status.PREEMPTING: {} break;
           case feedback->status.RECALLING: {} break;
           case feedback->status.RECALLED: {} break;
           case feedback->status.LOST: {} break;
           default: {} break;
        }
    }
}

bool rosNavigator::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        yarp::rosmsg::geometry_msgs::Pose gpose;
        gpose.position.x = loc.x;
        gpose.position.y = loc.y;
        gpose.position.z = 0;
        yarp::math::Quaternion q;
        yarp::sig::Vector v(4);
        v[0] = 0;
        v[1] = 0;
        v[2] = 1;
        v[3] = loc.theta*DEG2RAD;
        q.fromAxisAngle(v);
        gpose.orientation.x = q.x();
        gpose.orientation.y = q.y();
        gpose.orientation.z = q.z();
        gpose.orientation.w = q.w();
        m_current_goal = loc;
        //m_current_goal.map_id = "ros_map";

        if (1)
        {
            yarp::rosmsg::move_base_msgs::MoveBaseActionGoal& goal = m_rosPublisher_goal.prepare();
            goal.clear();
            goal.header.frame_id = m_abs_frame_id;
            goal.header.seq = 0;
            goal.header.stamp.sec = 0;
            goal.header.stamp.nsec = 0;
            goal.goal_id.id = "goal_0";
            goal.goal.target_pose.header.frame_id = m_abs_frame_id;
            goal.goal.target_pose.header.seq = 0;
            goal.goal.target_pose.header.stamp.sec = 0;
            goal.goal.target_pose.header.stamp.nsec = 0;
            goal.goal.target_pose.pose = gpose;
            m_rosPublisher_goal.write();
        }
        else
        {
            yarp::rosmsg::geometry_msgs::PoseStamped& pos = m_rosPublisher_simple_goal.prepare();
            pos.clear();
            pos.header.frame_id = m_abs_frame_id;
            pos.header.seq = 0;
            pos.header.stamp.sec = 0;
            pos.header.stamp.nsec = 0;
            pos.pose = gpose;
            m_rosPublisher_simple_goal.write();
        }
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - theta; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - 0; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    yError() << "applyVelocityCommand() not implemented in rosNavigator";
    return true;
}

bool rosNavigator::getNavigationStatus(yarp::dev::NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool rosNavigator::stopNavigation()
{
    yarp::rosmsg::actionlib_msgs::GoalID& goal_id =  m_rosPublisher_cancel.prepare();
    goal_id.clear();
    goal_id.id = "goal_0";
    m_rosPublisher_cancel.write();

    m_navigation_status = yarp::dev::navigation_status_idle;
    return true;
}

bool rosNavigator::getAbsoluteLocationOfCurrentTarget(Map2DLocation& target)
{
    target = m_current_goal;
    return true;
}

bool rosNavigator::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    x = m_current_goal.x- m_current_position.x; // @@@THIS NEEDS TO BE FIXED
    y = m_current_goal.y - m_current_position.y; // @@@THIS NEEDS TO BE FIXED
    theta = m_current_goal.theta - m_current_position.theta; // @@@THIS NEEDS TO BE FIXED
    return true;
}

bool rosNavigator::suspendNavigation(double time)
{
    yError() << "Unable to pause current navigation task";
    return false;
}

bool rosNavigator::resumeNavigation()
{
    yError() << "Unable to resume any paused navigation task";
    return false;
}

bool rosNavigator::getAllNavigationWaypoints(Map2DPath& waypoints)
{
    yDebug() << "Not yet implemented";
    return false;
}

/**
* Returns the current waypoint pursued by the navigation algorithm
* @param curr_waypoint the current waypoint pursued by the navigation algorithm
* @return true/false
*/
bool rosNavigator::getCurrentNavigationWaypoint(Map2DLocation& curr_waypoint)
{
    yDebug() << "Not yet implemented";
    return false;
}

bool rosNavigator::getCurrentNavigationMap(yarp::dev::NavigationMapTypeEnum map_type, MapGrid2D& map)
{
    if (map_type == yarp::dev::NavigationMapTypeEnum::global_map)
    {
        map = m_global_map;
        return true;
    }
    else if (map_type == yarp::dev::NavigationMapTypeEnum::local_map)
    {
        map = m_local_map;
        return true;
    }
    yError() << "rosNavigator::getCurrentNavigationMap invalid type";
    return false;
}

bool rosNavigator::recomputeCurrentNavigationPath()
{
    if (m_navigation_status == yarp::dev::navigation_status_moving)
    {
        yDebug() << "Not yet implemented";
        return false;
    }
    yError() << "Unable to recompute path. Navigation task not assigned yet.";
    return false;
}

std::string rosNavigator::getStatusAsString(NavigationStatusEnum status)
{
    if (status == navigation_status_idle) return std::string("navigation_status_idle");
    else if (status == navigation_status_moving) return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle) return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached) return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted) return std::string("navigation_status_aborted");
    else if (status == navigation_status_failing) return std::string("navigation_status_failing");
    else if (status == navigation_status_paused) return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move) return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking) return std::string("navigation_status_thinking");
    else if (status == navigation_status_error) return std::string("navigation_status_error");
    return std::string("navigation_status_error");
}