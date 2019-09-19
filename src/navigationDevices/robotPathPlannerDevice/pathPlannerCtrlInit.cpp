/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "pathPlannerCtrl.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

PlannerThread::PlannerThread(double _period, Searchable &_cfg) :
        PeriodicThread(_period), m_cfg(_cfg)
{
    m_planner_status = navigation_status_idle;
    m_inner_status = navigation_status_idle;
    m_loc_timeout_counter = 0;
    m_laser_timeout_counter = 0;
    m_inner_status_timeout_counter = 0;
    m_goal_tolerance_lin = 0.05;
    m_goal_tolerance_ang = 0.6;
    m_waypoint_tolerance_lin = 0.05;
    m_waypoint_tolerance_ang = 0.6;
    m_goal_max_lin_speed = 0.9;
    m_goal_max_ang_speed = 10.0;
    m_goal_min_lin_speed = 0.0;
    m_goal_min_ang_speed = 0.0;
    m_waypoint_max_lin_speed = 0.9;
    m_waypoint_max_ang_speed = 10.0;
    m_waypoint_min_lin_speed = 0.0;
    m_waypoint_min_ang_speed = 0.0;
    m_use_optimized_path = true;
    m_current_path = &m_computed_simplified_path;
    m_min_waypoint_distance = 0;
    m_iLaser = 0;
    m_iLoc = 0;
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    m_enable_try_recovery=false;
    m_stats_time_curr = yarp::os::Time::now();
    m_stats_time_last = yarp::os::Time::now();
    m_iInnerNav_ctrl = 0;
    m_iInnerNav_target = 0;
    m_force_map_reload = false;
    m_navigation_started_at_timeX = 0;
    m_final_goal_reached_at_timeX = 0;
}

bool PlannerThread::threadInit()
{
    //read configuration parameters
    std::string debug_rf = m_cfg.toString();
    Bottle navigation_group = m_cfg.findGroup("NAVIGATION");
    if (navigation_group.isNull())
    {
        yError() << "Missing NAVIGATION group!";
        return false;
    }
    if (navigation_group.check("waypoint_tolerance_lin")) { m_waypoint_tolerance_lin = navigation_group.find("waypoint_tolerance_lin").asDouble(); }
    else { yError() << "Missing waypoint_tolerance_lin parameter"; return false; }
    if (navigation_group.check("waypoint_tolerance_ang")) { m_waypoint_tolerance_ang = navigation_group.find("waypoint_tolerance_ang").asDouble(); }
    else { yError() << "Missing waypoint_tolerance_ang parameter"; return false; }
    if (navigation_group.check("goal_tolerance_lin")) { m_goal_tolerance_lin = navigation_group.find("goal_tolerance_lin").asDouble(); }
    else { yError() << "Missing goal_tolerance_lin parameter"; return false; }
    if (navigation_group.check("goal_tolerance_ang")) { m_goal_tolerance_ang = navigation_group.find("goal_tolerance_ang").asDouble(); }
    else { yError() << "Missing goal_tolerance_ang parameter"; return false; }
    if (navigation_group.check("use_optimized_path")) { int p = navigation_group.find("use_optimized_path").asInt(); m_use_optimized_path = (p == 1); }
    else { yError() << "Missing use_optimized_path parameter"; return false; }
    if (navigation_group.check("waypoint_max_lin_speed")) { m_waypoint_max_lin_speed = navigation_group.find("waypoint_max_lin_speed").asDouble(); }
    else { yError() << "Missing waypoint_max_lin_speed parameter"; return false; }
    if (navigation_group.check("waypoint_max_ang_speed")) { m_waypoint_max_ang_speed = navigation_group.find("waypoint_max_ang_speed").asDouble(); }
    else { yError() << "Missing waypoint_max_ang_speed parameter"; return false; }
    if (navigation_group.check("waypoint_min_lin_speed")) { m_waypoint_min_lin_speed = navigation_group.find("waypoint_min_lin_speed").asDouble(); }
    else { yError() << "Missing waypoint_min_lin_speed parameter"; return false; }
    if (navigation_group.check("waypoint_min_ang_speed")) { m_waypoint_min_ang_speed = navigation_group.find("waypoint_min_ang_speed").asDouble(); }
    else { yError() << "Missing waypoint_min_ang_speed parameter"; return false; }
    if (navigation_group.check("waypoint_lin_speed_gain")) { m_waypoint_lin_gain = navigation_group.find("waypoint_lin_speed_gain").asDouble(); }
    else { yError() << "Missing waypoint_lin_speed_gain parameter"; return false; }
    if (navigation_group.check("waypoint_ang_speed_gain")) { m_waypoint_ang_gain = navigation_group.find("waypoint_ang_speed_gain").asDouble(); }
    else { yError() << "Missing waypoint_ang_speed_gain parameter"; return false; }
    if (navigation_group.check("goal_max_lin_speed")) { m_goal_max_lin_speed = navigation_group.find("goal_max_lin_speed").asDouble(); }
    else { yError() << "Missing goal_max_lin_speed parameter"; return false; }
    if (navigation_group.check("goal_max_ang_speed")) { m_goal_max_ang_speed = navigation_group.find("goal_max_ang_speed").asDouble(); }
    else { yError() << "Missing goal_max_ang_speed parameter"; return false; }
    if (navigation_group.check("goal_min_lin_speed")) { m_goal_min_lin_speed = navigation_group.find("goal_min_lin_speed").asDouble(); }
    else { yError() << "Missing goal_min_lin_speed parameter"; return false; }
    if (navigation_group.check("goal_min_ang_speed")) { m_goal_min_ang_speed = navigation_group.find("goal_min_ang_speed").asDouble(); }
    else { yError() << "Missing goal_min_ang_speed parameter"; return false; }
    if (navigation_group.check("goal_lin_speed_gain")) { m_goal_lin_gain = navigation_group.find("goal_lin_speed_gain").asDouble(); }
    else { yError() << "Missing goal_lin_speed_gain parameter"; return false; }
    if (navigation_group.check("goal_ang_speed_gain")) { m_goal_ang_gain = navigation_group.find("goal_ang_speed_gain").asDouble(); }
    else { yError() << "Missing goal_ang_speed_gain parameter"; return false; }
    if (navigation_group.check("min_waypoint_distance")) { m_min_waypoint_distance = navigation_group.find("min_waypoint_distance").asInt(); }
    else { yError() << "Missing min_waypoint_distance parameter"; return false; }
    if (navigation_group.check("enable_try_recovery")) { m_enable_try_recovery = (navigation_group.find("enable_try_recovery").asInt() == 1); }
    else { yError() << "Missing enable_try_recovery parameter"; return false; }

    Bottle general_group = m_cfg.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError() << "Missing GENERAL group!";
        return false;
    }

    Bottle geometry_group = m_cfg.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }
    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yError() << "Missing LOCALIZATION group!";
        return false;
    }

    bool ff = geometry_group.check("robot_radius");
    ff &= geometry_group.check("laser_pos_x");
    ff &= geometry_group.check("laser_pos_y");
    ff &= geometry_group.check("laser_pos_theta");

    if (localization_group.check("robot_frame_id")) { m_frame_robot_id = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id")) { m_frame_map_id = localization_group.find("map_frame_id").asString(); }

    if (ff)
    {
        m_robot_radius = geometry_group.find("robot_radius").asDouble();
        m_robot_laser_x = geometry_group.find("laser_pos_x").asDouble();
        m_robot_laser_y = geometry_group.find("laser_pos_y").asDouble();
        m_robot_laser_t = geometry_group.find("laser_pos_theta").asDouble();
    }
    else
    {
        yError() << "Invalid/missing parameter in ROBOT_GEOMETRY group";
        return false;
    }

    //open module ports
    bool ret = true;
    string localName = "/robotPathPlanner";
    ret &= m_port_status_output.open((localName + "/plannerStatus:o").c_str());
    ret &= m_port_commands_output.open((localName + "/commands:o").c_str());
    ret &= m_port_map_output.open((localName + "/map:o").c_str());
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    //localization
    {
        Property loc_options;
        loc_options.put("device", "localization2DClient");
        loc_options.put("local", "/robotPathPlanner/localizationClient");
        loc_options.put("remote", "/localizationServer");
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

    //open the map interface
    {
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
    }

    //open the laser interface
    {
        Bottle laserBottle = m_cfg.findGroup("LASER");
        if (laserBottle.isNull())
        {
            yError("LASER group not found,closing");
            return false;
        }
        if (laserBottle.check("laser_port") == false)
        {
            yError("laser_port param not found,closing");
            return false;
        }
        string laser_remote_port = laserBottle.find("laser_port").asString();

        Property las_options;
        las_options.put("device", "Rangefinder2DClient");
        las_options.put("local", "/robotPathPlanner/laser:i");
        las_options.put("remote", laser_remote_port);
        if (m_pLas.open(las_options) == false)
        {
            yError() << "Unable to open laser driver";
            return false;
        }
        m_pLas.view(m_iLaser);
        if (m_iLaser == 0)
        {
            yError() << "Unable to open laser interface";
            return false;
        }
        if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
        {
            yError() << "Unable to obtain laser scan limits";
            return false;
        }
        m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);
    }


    //open the local navigator
    {
        Bottle innerNavigation_group = m_cfg.findGroup("INTERNAL_NAVIGATOR");
        if (innerNavigation_group.isNull())
        {
            yError() << "Missing INTERNAL_NAVIGATOR group!";
            return false;
        }
        bool nn = innerNavigation_group.check("plugin");
        nn &= innerNavigation_group.check("context");
        nn &= innerNavigation_group.check("from");
        if (!nn)
        {
            yError() << "Invalid/missing parameter in INTERNAL_NAVIGATOR group";
            return false;
        }
        m_localNavigatorPlugin_name = innerNavigation_group.find("plugin").asString();
        std::string inner_ctex = innerNavigation_group.find("context").asString();
        std::string inner_file = innerNavigation_group.find("from").asString();

        Property innerNav_options;
        innerNav_options.put("from", inner_file);
        innerNav_options.put("context", inner_ctex);
        innerNav_options.put("device", m_localNavigatorPlugin_name);
        if (m_pInnerNav.open(innerNav_options) == false)
        {
            yError() << "Unable to open local Navigator plugin:" << m_localNavigatorPlugin_name;
            return false;
        }
        m_pInnerNav.view(m_iInnerNav_target);
        if (m_iInnerNav_target == 0)
        {
            yError() << "Unable to open m_iInnerNav_target interface";
            return false;
        }
        m_pInnerNav.view(m_iInnerNav_ctrl);
        if (m_iInnerNav_ctrl == 0)
        {
            yError() << "Unable to open m_iInnerNav_ctrl interface";
            return false;
        }
    }
    return true;
}

void PlannerThread :: threadRelease()
{
    if (m_pLoc.isValid()) m_pLoc.close();
    if (m_ptf.isValid()) m_ptf.close();
    if (m_pLas.isValid()) m_pLas.close();
    m_port_map_output.interrupt();
    m_port_map_output.close();
    m_port_status_output.interrupt();
    m_port_status_output.close();
    m_port_commands_output.interrupt();
    m_port_commands_output.close();
}


