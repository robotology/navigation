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
using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(PATHPLAN_INIT, "navigation.devices.robotPathPlanner.init")

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
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    m_enable_try_recovery=false;
    m_stats_time_curr = yarp::os::Time::now();
    m_stats_time_last = yarp::os::Time::now();
    m_force_map_reload = false;
    m_navigation_started_at_timeX = 0;
    m_final_goal_reached_at_timeX = 0;
}

bool PlannerThread::threadInit()
{
	//default values
	string localName = "/robotPathPlanner";
    string localizationServer_name = "/localizationServer";
    string mapServer_name = "/mapServer";
    
    //read configuration parameters
    std::string debug_rf = m_cfg.toString();
    Bottle navigation_group = m_cfg.findGroup("NAVIGATION");
    if (navigation_group.isNull())
    {
        yCError(PATHPLAN_INIT) << "Missing NAVIGATION group!";
        return false;
    }
    if (navigation_group.check("waypoint_tolerance_lin")) { m_waypoint_tolerance_lin = navigation_group.find("waypoint_tolerance_lin").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_tolerance_lin parameter"; return false; }
    if (navigation_group.check("waypoint_tolerance_ang")) { m_waypoint_tolerance_ang = navigation_group.find("waypoint_tolerance_ang").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_tolerance_ang parameter"; return false; }
    if (navigation_group.check("goal_tolerance_lin")) { m_goal_tolerance_lin = navigation_group.find("goal_tolerance_lin").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_tolerance_lin parameter"; return false; }
    if (navigation_group.check("goal_tolerance_ang")) { m_goal_tolerance_ang = navigation_group.find("goal_tolerance_ang").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_tolerance_ang parameter"; return false; }
    if (navigation_group.check("use_optimized_path")) { int p = navigation_group.find("use_optimized_path").asInt32(); m_use_optimized_path = (p == 1); }
    else { yCError(PATHPLAN_INIT) << "Missing use_optimized_path parameter"; return false; }
    if (navigation_group.check("waypoint_max_lin_speed")) { m_waypoint_max_lin_speed = navigation_group.find("waypoint_max_lin_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_max_lin_speed parameter"; return false; }
    if (navigation_group.check("waypoint_max_ang_speed")) { m_waypoint_max_ang_speed = navigation_group.find("waypoint_max_ang_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_max_ang_speed parameter"; return false; }
    if (navigation_group.check("waypoint_min_lin_speed")) { m_waypoint_min_lin_speed = navigation_group.find("waypoint_min_lin_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_min_lin_speed parameter"; return false; }
    if (navigation_group.check("waypoint_min_ang_speed")) { m_waypoint_min_ang_speed = navigation_group.find("waypoint_min_ang_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_min_ang_speed parameter"; return false; }
    if (navigation_group.check("waypoint_lin_speed_gain")) { m_waypoint_lin_gain = navigation_group.find("waypoint_lin_speed_gain").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_lin_speed_gain parameter"; return false; }
    if (navigation_group.check("waypoint_ang_speed_gain")) { m_waypoint_ang_gain = navigation_group.find("waypoint_ang_speed_gain").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing waypoint_ang_speed_gain parameter"; return false; }
    if (navigation_group.check("goal_max_lin_speed")) { m_goal_max_lin_speed = navigation_group.find("goal_max_lin_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_max_lin_speed parameter"; return false; }
    if (navigation_group.check("goal_max_ang_speed")) { m_goal_max_ang_speed = navigation_group.find("goal_max_ang_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_max_ang_speed parameter"; return false; }
    if (navigation_group.check("goal_min_lin_speed")) { m_goal_min_lin_speed = navigation_group.find("goal_min_lin_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_min_lin_speed parameter"; return false; }
    if (navigation_group.check("goal_min_ang_speed")) { m_goal_min_ang_speed = navigation_group.find("goal_min_ang_speed").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_min_ang_speed parameter"; return false; }
    if (navigation_group.check("goal_lin_speed_gain")) { m_goal_lin_gain = navigation_group.find("goal_lin_speed_gain").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_lin_speed_gain parameter"; return false; }
    if (navigation_group.check("goal_ang_speed_gain")) { m_goal_ang_gain = navigation_group.find("goal_ang_speed_gain").asFloat64(); }
    else { yCError(PATHPLAN_INIT) << "Missing goal_ang_speed_gain parameter"; return false; }
    if (navigation_group.check("min_waypoint_distance")) { m_min_waypoint_distance = navigation_group.find("min_waypoint_distance").asInt32(); }
    else { yCError(PATHPLAN_INIT) << "Missing min_waypoint_distance parameter"; return false; }
    if (navigation_group.check("enable_try_recovery")) { m_enable_try_recovery = (navigation_group.find("enable_try_recovery").asInt32() == 1); }
    else { yCError(PATHPLAN_INIT) << "Missing enable_try_recovery parameter"; return false; }

    Bottle general_group = m_cfg.findGroup("PATHPLANNER_GENERAL");
    if (general_group.isNull())
    {
        yCError(PATHPLAN_INIT) << "Missing PATHPLANNER_GENERAL group!";
        return false;
    }

    Bottle geometry_group = m_cfg.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError(PATHPLAN_INIT) << "Missing ROBOT_GEOMETRY group!";
        return false;
    }
    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yCError(PATHPLAN_INIT) << "Missing LOCALIZATION group!";
        return false;
    }

    if (localization_group.check("robot_frame_id")) { m_frame_robot_id = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id")) { m_frame_map_id = localization_group.find("map_frame_id").asString(); }
    if (localization_group.check("localizationServer_name")) localizationServer_name = localization_group.find("localizationServer_name").asString();
    if (localization_group.check("mapServer_name")) mapServer_name = localization_group.find("mapServer_name").asString();
    if (general_group.check("name")) localName = general_group.find("name").asString();
    
    bool ff = geometry_group.check("robot_radius");
    ff &= geometry_group.check("laser_pos_x");
    ff &= geometry_group.check("laser_pos_y");
    ff &= geometry_group.check("laser_pos_theta");
    if (ff)
    {
        m_robot_radius = geometry_group.find("robot_radius").asFloat64();
        m_robot_laser_x = geometry_group.find("laser_pos_x").asFloat64();
        m_robot_laser_y = geometry_group.find("laser_pos_y").asFloat64();
        m_robot_laser_t = geometry_group.find("laser_pos_theta").asFloat64();
    }
    else
    {
        yCError(PATHPLAN_INIT) << "Invalid/missing parameter in ROBOT_GEOMETRY group";
        return false;
    }
    
    //open module ports
    bool ret = true;
    ret &= m_port_status_output.open((localName + "/plannerStatus:o").c_str());
    ret &= m_port_commands_output.open((localName + "/commands:o").c_str());
    ret &= m_port_map_output.open((localName + "/map:o").c_str());
    if (ret == false)
    {
        yCError(PATHPLAN_INIT) << "Unable to open module ports";
        return false;
    }

    //localization
    {
        Property loc_options;
        loc_options.put("device", "localization2DClient");
        loc_options.put("local", localName+"/localizationClient");
        loc_options.put("remote", localizationServer_name);
        if (m_pLoc.open(loc_options) == false)
        {
            yCError(PATHPLAN_INIT) << "Unable to open localization driver";
            return false;
        }
        m_pLoc.view(m_iLoc);
        if (m_pLoc.isValid() == false || m_iLoc == nullptr)
        {
            yCError(PATHPLAN_INIT) << "Unable to view localization interface";
            return false;
        }
    }

    //open the map interface
    {
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", localName); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", mapServer_name);
        if (m_pMap.open(map_options) == false)
        {
            yCError(PATHPLAN_INIT) << "Unable to open mapClient";
            return false;
        }
        m_pMap.view(m_iMap);
        if (m_iMap == nullptr)
        {
            yCError(PATHPLAN_INIT) << "Unable to open map interface";
            return false;
        }
    }

    //open the laser interface
    {
        Bottle laserBottle = m_cfg.findGroup("LASER");
        if (laserBottle.isNull())
        {
            yCError(PATHPLAN_INIT,"LASER group not found,closing");
            return false;
        }
        if (laserBottle.check("laser_port") == false)
        {
            yCError(PATHPLAN_INIT,"laser_port param not found,closing");
            return false;
        }
        string laser_remote_port = laserBottle.find("laser_port").asString();

        Property las_options;
        las_options.put("device", "Rangefinder2DClient");
        las_options.put("local", localName+"/laser:i");
        las_options.put("remote", laser_remote_port);
        if (m_pLas.open(las_options) == false)
        {
            yCError(PATHPLAN_INIT) << "Unable to open laser driver";
            return false;
        }
        m_pLas.view(m_iLaser);
        if (m_iLaser == nullptr)
        {
            yCError(PATHPLAN_INIT) << "Unable to open laser interface";
            return false;
        }
        if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
        {
            yCError(PATHPLAN_INIT) << "Unable to obtain laser scan limits";
            return false;
        }
        m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);
    }


    //open the local navigator
    {
        Bottle innerNavigation_group = m_cfg.findGroup("INTERNAL_NAVIGATOR");
        if (innerNavigation_group.isNull())
        {
            yCError(PATHPLAN_INIT) << "Missing INTERNAL_NAVIGATOR group!";
            return false;
        }
        bool nn = innerNavigation_group.check("plugin");
        nn &= innerNavigation_group.check("context");
        nn &= innerNavigation_group.check("from");
        if (!nn)
        {
            yCError(PATHPLAN_INIT) << "Invalid/missing parameter in INTERNAL_NAVIGATOR group";
            return false;
        }
        m_localNavigatorPlugin_name = innerNavigation_group.find("plugin").asString();
        std::string inner_ctex = innerNavigation_group.find("context").asString();
        std::string inner_file = innerNavigation_group.find("from").asString();

        yarp::os::ResourceFinder rf;
        rf.setDefaultConfigFile(inner_file);
        rf.setDefaultContext(inner_ctex);
        rf.configure(0, nullptr);

        Property innerNav_options;
        string tmp = rf.toString();
        innerNav_options.fromString(tmp);
        innerNav_options.put("device", m_localNavigatorPlugin_name);
        yCDebug(PATHPLAN_INIT) << "Opening local navigator" << m_localNavigatorPlugin_name << "with params: "<< " --context" << inner_ctex << " --from" << inner_file;
        yCDebug(PATHPLAN_INIT) << "Full configuration:" << innerNav_options.toString();

        if (m_pInnerNav.open(innerNav_options) == false)
        {
            yCError(PATHPLAN_INIT) << "Unable to open local Navigator plugin:" << m_localNavigatorPlugin_name;
            return false;
        }
        m_pInnerNav.view(m_iInnerNav_target);
        if (m_iInnerNav_target == nullptr)
        {
            yCError(PATHPLAN_INIT) << "Unable to open m_iInnerNav_target interface";
            return false;
        }
        m_pInnerNav.view(m_iInnerNav_ctrl);
        if (m_iInnerNav_ctrl == nullptr)
        {
            yCError(PATHPLAN_INIT) << "Unable to open m_iInnerNav_ctrl interface";
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


