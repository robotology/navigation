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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

isaacNavigator::isaacNavigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = navigation_status_idle;
    m_local_name_prefix = "/isaacNavigator";
    m_map_dd_enable = false;
    m_localization_dd_enable = false;
    m_port_navigation_status_name= m_local_name_prefix+string("/status:i");
    m_port_navigation_command_name= m_local_name_prefix+ string("/command:o");
    m_port_global_trajectory_name = m_local_name_prefix + string("/global_trajectory:i");
    m_port_local_trajectory_name = m_local_name_prefix + string("/local_trajectory:i");
}

bool isaacNavigator::open(yarp::os::Searchable& config)
{
    m_iLoc = nullptr;
    m_iMap = nullptr;
    //ports for communication with isaac
    m_port_navigation_status.open(m_port_navigation_status_name);
    m_port_navigation_command.open(m_port_navigation_command_name);
    m_port_global_trajectory.open(m_port_global_trajectory_name);
    m_port_local_trajectory.open(m_port_local_trajectory_name);
    if (config.check("autoconnect"))
    {
       bool ret = true;
       ret &= yarp::os::Network::connect("/yarpbridge/goalfeedback:o", m_port_navigation_status_name);
       ret &= yarp::os::Network::connect(m_port_navigation_command_name,"/yarpbridge/goal_cmd:i");
       ret &= yarp::os::Network::connect("/yarpbridge/plan:o", m_port_global_trajectory_name);
       ret &= yarp::os::Network::connect("/yarpbridge/differential_trajectory_plan:o", m_port_local_trajectory_name);
       if (!ret)
       {
           yError() << "Autoconnect failed";
           return false;
       }
    }

    this->start();
    return true;
}

bool isaacNavigator::close()
{
    m_port_navigation_status.interrupt();
    m_port_navigation_status.close();
    m_port_navigation_command.interrupt();
    m_port_navigation_command.close();
    m_port_global_trajectory.interrupt();
    m_port_global_trajectory.close();
    m_port_local_trajectory.interrupt();
    m_port_local_trajectory.close();
    return true;
}

bool isaacNavigator::threadInit()
{
    //init values
    m_remote_localization = "/localizationServer";
    m_remote_mapserver = "/mapServer";
    m_abs_frame_id = "/map";
    m_map_name = "isaacMap";

    if (m_localization_dd_enable)
    {
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
    }

    //if (m_map_dd_enable)
    //map server is needed because navigationGUI will ask a map to us
    {
        //map_server
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", m_local_name_prefix); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", m_remote_mapserver);
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
        yInfo() << "Asking for map..." << m_map_name;
        bool b = m_iMap->get_map(m_map_name,m_global_map);
        m_global_map.crop(-1,-1,-1,-1);
        if (b)
        {
            yInfo() << "map name:" << m_map_name << " received";
        }
        else
        {
            yError() << "map name:" << m_map_name << " not found";
        }
    }

    m_stats_time_curr = yarp::os::Time::now();
    m_stats_time_last = yarp::os::Time::now();

    return true;
}

void isaacNavigator::threadRelease()
{
    if (m_map_dd_enable)
    {
        m_pMap.close();
    }

    if (m_localization_dd_enable)
    {
        m_pLoc.close();
    }
}

void isaacNavigator::run()
{
    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        m_stats_time_last = m_stats_time_curr;
        bool err = false;
        /*
        if (m_loc_timeout_counter > TIMEOUT_MAX)
        {
            yError(" timeout, no localization data received!\n");
            err = true;
        }
        */
        if (err == false)
            yInfo() << "isaacNavigator running, ALL ok. Navigation status:" << getStatusAsString(m_navigation_status);
    }

    //robot position
    if (m_iLoc)
    {
        bool b1 = m_iLoc->getCurrentPosition(m_current_position);
    }

    // ISAAC feedback
    yarp::os::Bottle* plan = m_port_global_trajectory.read(false);
    if (plan)
    {
        m_global_plan.clear();
        string frame = plan->get(0).asString();
        for (size_t i = 1; i < plan->size(); i++)
        {
            Bottle* list_traj = plan->get(1).asList();
            Map2DLocation loc;
            loc.map_id = "";
            loc.x = list_traj->get(0).asFloat64();
            loc.y = list_traj->get(1).asFloat64();
            loc.theta = list_traj->get(2).asFloat64();
            m_global_plan.push_back(loc);
        }
    }

    yarp::os::Bottle* trajectory = m_port_local_trajectory.read(false);
    if (trajectory)
    {
       m_local_plan.clear();
       string frame = trajectory->get(0).asString();
       for (size_t i = 1; i < trajectory->size(); i++)
       {
           Bottle* list_traj = trajectory->get(1).asList();
           Map2DLocation loc;
           loc.map_id = "";
           loc.x = list_traj->get(0).asFloat64();
           loc.y = list_traj->get(1).asFloat64();
           loc.theta = list_traj->get(2).asFloat64();
           m_local_plan.push_back(loc);
       }
    }

    yarp::os::Bottle* feedback = m_port_navigation_status.read(false);
    if (feedback)
    {
        Bottle* goal_rel_loc     = feedback->get(0).asList();
        bool    has_goal         = feedback->get(1).asBool();
        bool    is_arrived       = feedback->get(2).asBool();
        bool    is_stationary    = feedback->get(3).asBool();
    }

    //statemachine
    switch (m_navigation_status)
    {
        case NavigationStatusEnum::navigation_status_aborted:
        case NavigationStatusEnum::navigation_status_error:
        break;

        case NavigationStatusEnum::navigation_status_idle:
        case NavigationStatusEnum::navigation_status_moving:
        case NavigationStatusEnum::navigation_status_waiting_obstacle:
        case NavigationStatusEnum::navigation_status_goal_reached:
        case NavigationStatusEnum::navigation_status_failing:
        case NavigationStatusEnum::navigation_status_paused:
        case NavigationStatusEnum::navigation_status_preparing_before_move:
        case NavigationStatusEnum::navigation_status_thinking:

        break;
    }
}

bool isaacNavigator::gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_idle)
    {
        auto& cmd = m_port_navigation_command.prepare();
		cmd.clear();
		//map location
		yarp::os::Bottle& locb = cmd.addList();
		locb.addString(loc.map_id);
		locb.addFloat64(loc.x);
		locb.addFloat64(loc.y);
		locb.addFloat64(loc.theta );
		//goal tolerance
		cmd.addFloat64(0.2);
		//stop command
		//cmd.addBool(false); //@@@@
		cmd.addInt(0);
		//reference frame
		cmd.addString("world");
		m_port_navigation_command.write();
        m_navigation_status = yarp::dev::Nav2D::navigation_status_moving;
        m_current_goal = loc;
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
    auto& cmd= m_port_navigation_command.prepare();
    cmd.clear();
    //map location
    yarp::os::Bottle& loc = cmd.addList();
    loc.addString("");
    loc.addFloat64(0.0);
    loc.addFloat64(0.0);
    loc.addFloat64(0.0);
    //goal tolerance
    cmd.addFloat64(0.0);
    //stop command
    //cmd.addBool(true); //@@@@
    cmd.addInt(1);
    //reference frame
    cmd.addString("stop");
    m_port_navigation_command.write();
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
    auto& cmd = m_port_navigation_command.prepare();
    cmd.clear();
    //map location
    yarp::os::Bottle& loc = cmd.addList();
    loc.addString("");
    loc.addFloat64(0.0);
    loc.addFloat64(0.0);
    loc.addFloat64(0.0);
    //goal tolerance
    cmd.addFloat64(0.0);
    //stop command
    //cmd.addBool(true); //@@@@
    cmd.addInt(1);
    //reference frame
    cmd.addString("stop");
    m_port_navigation_command.write();
    m_navigation_status = yarp::dev::Nav2D::navigation_status_paused;
    return true;
}

bool isaacNavigator::resumeNavigation()
{
    if (m_navigation_status == yarp::dev::Nav2D::navigation_status_paused)
    {
        auto& cmd = m_port_navigation_command.prepare();
		cmd.clear();
		//map location
		yarp::os::Bottle& locb = cmd.addList();
		locb.addString(m_current_goal.map_id);
		locb.addFloat64(m_current_goal.x);
		locb.addFloat64(m_current_goal.y);
		locb.addFloat64(m_current_goal.theta );
		//goal tolerance
		cmd.addFloat64(0.2);
		//stop command
		//cmd.addBool(false); //@@@@
		cmd.addInt(0);
		//reference frame
		cmd.addString("world");
		m_port_navigation_command.write();
        m_navigation_status = yarp::dev::Nav2D::navigation_status_moving;
        return true;
    }
    yError() << "No navigation tasks to resume";
    return false;
}

bool isaacNavigator::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    yError() << "applyVelocityCommand() not implemented";
    return true;
}

bool isaacNavigator::getAllNavigationWaypoints(yarp::dev::Nav2D::Map2DPath& waypoints)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    waypoints = m_global_plan;
    return true;
}


bool isaacNavigator::getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    curr_waypoint = m_current_waypoint;
    return true;
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
    if      (status == navigation_status_idle)     return std::string("navigation_status_idle");
    else if (status == navigation_status_moving)   return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle) return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached)     return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted)  return std::string("navigation_status_aborted");
    else if (status == navigation_status_failing)  return std::string("navigation_status_failing");
    else if (status == navigation_status_paused)   return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move) return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking) return std::string("navigation_status_thinking");
    else if (status == navigation_status_error)    return std::string("navigation_status_error");
    return std::string("navigation_status_error");
}
