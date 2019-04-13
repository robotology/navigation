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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include <cmath>
#include <yarp/math/Math.h>
#include "isaacNavigator.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

isaacNavigator::isaacNavigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = navigation_status_idle;
    m_local_name_prefix = "/rosNavigator";
    m_remote_localization = "/localizationServer";
//    m_abs_frame_id = "/odom";
    m_abs_frame_id = "/map";
}

bool isaacNavigator::open(yarp::os::Searchable& config)
{
    //ISAAC
    //@@@

    this->start();
    return true;
}

bool isaacNavigator::close()
{
    return true;
}

bool isaacNavigator::threadInit()
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

void isaacNavigator::threadRelease()
{
    m_pLoc.close();
}

void isaacNavigator::run()
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
            yInfo() << "isaacNavigator running, ALL ok. Navigation status:" << getStatusAsString(m_navigation_status);
    }

    bool b1 = m_iLoc->getCurrentPosition(m_current_position);

    //@@@
}

bool isaacNavigator::gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_idle)
    {
        //@@@
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool isaacNavigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_idle)
    {
        yarp::dev::Nav2D::Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - theta; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool isaacNavigator::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_idle)
    {
        yarp::dev::Nav2D::Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - 0; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool isaacNavigator::getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool isaacNavigator::stopNavigation()
{
    //@@@

    m_navigation_status = yarp::dev::Nav2D::navigation_status_idle;
    return true;
}

bool isaacNavigator::getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target)
{
    target = m_current_goal;
    return true;
}

bool isaacNavigator::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    x = m_current_goal.x- m_current_position.x; // @@@THIS NEEDS TO BE FIXED
    y = m_current_goal.y - m_current_position.y; // @@@THIS NEEDS TO BE FIXED
    theta = m_current_goal.theta - m_current_position.theta; // @@@THIS NEEDS TO BE FIXED
    return true;
}

bool isaacNavigator::suspendNavigation(double time)
{
    yError() << "Unable to pause current navigation task";
    return false;
}

bool isaacNavigator::resumeNavigation()
{
    yError() << "Unable to resume any paused navigation task";
    return false;
}

bool isaacNavigator::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    yError() << "applyVelocityCommand() not implemented";
    return true;
}

bool isaacNavigator::getAllNavigationWaypoints(yarp::dev::Nav2D::Map2DPath& waypoints)
{
    yDebug() << "Not yet implemented";
    return false;
}

/**
* Returns the current waypoint pursued by the navigation algorithm
* @param curr_waypoint the current waypoint pursued by the navigation algorithm
* @return true/false
*/
bool isaacNavigator::getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint)
{
    yDebug() << "Not yet implemented";
    return false;
}

bool isaacNavigator::getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map)
{
    if (map_type == yarp::dev::Nav2D::NavigationMapTypeEnum::global_map)
    {
        map = m_global_map;
        return true;
    }
    else if (map_type == yarp::dev::Nav2D::NavigationMapTypeEnum::local_map)
    {
        map = m_local_map;
        return true;
    }
    yError() << "isaacNavigator::getCurrentNavigationMap invalid type";
    return false;
}

bool isaacNavigator::recomputeCurrentNavigationPath()
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_moving)
    {
        yDebug() << "Not yet implemented";
        return false;
    }
    yError() << "Unable to recompute path. Navigation task not assigned yet.";
    return false;
}

std::string isaacNavigator::getStatusAsString(NavigationStatusEnum status)
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