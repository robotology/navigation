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

#include "pathPlanner.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

PlannerThread::PlannerThread(unsigned int _period, ResourceFinder &_rf) :
        RateThread(_period), m_rf(_rf)
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
    m_max_lin_speed = 0.9;
    m_max_ang_speed = 10.0;
    m_min_lin_speed = 0.0;
    m_min_ang_speed = 0.0;
    m_use_optimized_path = true;
    m_current_path = &m_computed_simplified_path;
    m_min_waypoint_distance = 0;
    m_iLaser = 0;
    m_iTf = 0;
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    m_use_localization_from_port = false;
    m_use_localization_from_tf = false;
    m_imagemap_refresh_time = 0.033;
}

bool PlannerThread::threadInit()
{
    //read configuration parametes
    std::string debug_rf = m_rf.toString();
    if (m_rf.check("waypoint_tolerance_lin")) { m_waypoint_tolerance_lin = m_rf.find("waypoint_tolerance_lin").asDouble(); }
    if (m_rf.check("waypoint_tolerance_ang")) { m_waypoint_tolerance_ang = m_rf.find("waypoint_tolerance_ang").asDouble(); }
    if (m_rf.check("goal_tolerance_lin"))     { m_goal_tolerance_lin = m_rf.find("goal_tolerance_lin").asDouble(); }
    if (m_rf.check("goal_tolerance_ang"))     { m_goal_tolerance_ang = m_rf.find("goal_tolerance_ang").asDouble(); }
    if (m_rf.check("use_optimized_path"))     { int p = m_rf.find("use_optimized_path").asInt(); m_use_optimized_path = (p == 1); }
    if (m_rf.check("max_lin_speed"))          { m_max_lin_speed = m_rf.find("max_lin_speed").asDouble(); }
    if (m_rf.check("max_ang_speed"))          { m_max_ang_speed = m_rf.find("max_ang_speed").asDouble(); }
    if (m_rf.check("min_lin_speed"))          { m_min_lin_speed = m_rf.find("min_lin_speed").asDouble(); }
    if (m_rf.check("min_ang_speed"))          { m_min_ang_speed = m_rf.find("min_ang_speed").asDouble(); }
    if (m_rf.check("min_waypoint_distance"))  { m_min_waypoint_distance = m_rf.find("min_waypoint_distance").asInt(); }
    if (m_rf.check("publish_map_image_Hz"))  { m_imagemap_refresh_time = 1/(m_rf.find("publish_map_image_Hz").asDouble()); }

    Bottle geometry_group = m_rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }
    Bottle localization_group = m_rf.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yError() << "Missing LOCALIZATION group!";
        return false;
    }

    bool ff = geometry_group.check("robot_radius");
    ff &= geometry_group.check("laser_pos_x");
    ff &= geometry_group.check("laser_pos_y");
    ff &= geometry_group.check("laser_pos_theta");

    if (localization_group.check("use_localization_from_port")) { m_use_localization_from_port = (localization_group.find("use_localization_from_port").asInt() == 1); }
    if (localization_group.check("use_localization_from_tf"))   { m_use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }
    if (localization_group.check("robot_frame_id"))             { m_frame_robot_id = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id"))               { m_frame_map_id = localization_group.find("map_frame_id").asString(); }
    if (m_use_localization_from_port == true && m_use_localization_from_tf == true)
    {
        yError() << "`use_localization_from_tf` and `use_localization_from_port` cannot be true simulteneously!";
        return false;
    }

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
    string localName = "/robotPathPlanner";
    m_port_status_input.open((localName + "/navigationStatus:i").c_str());
    m_port_status_output.open((localName + "/plannerStatus:o").c_str());
    m_port_commands_output.open((localName + "/commands:o").c_str());
    m_port_map_output.open((localName + "/map:o").c_str());
    m_port_yarpview_target_input.open((localName + "/yarpviewTarget:i").c_str());
    m_port_yarpview_target_output.open((localName + "/yarpviewTarget:o").c_str());

    //localization
    if (m_use_localization_from_port)
    {
        m_port_localization_input.open((localName + "/localization:i").c_str());
    }

    if (m_use_localization_from_tf)
    {
        Property options;
        options.put("device", "transformClient");
        options.put("local", "/robotPathPlanner/localizationTfClient");
        options.put("remote", "/transformServer");
        if (m_ptf.open(options) == false)
        {
            yError() << "Unable to open transform client";
            return false;
        }
        m_ptf.view(m_iTf);
        if (m_ptf.isValid() == false || m_iTf == 0)
        {
            yError() << "Unable to view iTransform interface";
            return false;
        }
    }

    //open the map interface
    Property map_options;
    map_options.put("device", "map2DClient");
    map_options.put("local", "/robotPathPlanner/mapClient");
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

    //open the laser interface
    Bottle laserBottle = m_rf.findGroup("LASER");
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
    las_options.put("period", "10");
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

    //read the map
    string map_filename;
    //yarp::os::ResourceFinder mapFinder;
    //mapFinder.setDefaultContext("robot/maps");
    //mapFinder.configure(0, 0);
    //map_filename = mapFinder.getHomeContextPath().c_str() + string("/");
    //map_filename = map_filename + rf.find("map_file").asString().c_str();
    //map_filename = rf.find("map_file").asString().c_str();

    Bottle mapBottle = m_rf.findGroup("MAP");
    if (mapBottle.isNull())
    {
        yError("MAP group not found,closing");
        return false;
    }
    if (mapBottle.check("file_name")==false)
    {
        yError("map_file param not found,closing");
        return false;
    }
    map_filename = mapBottle.find("file_name").asString();

    if (!m_current_map.loadFromFile(map_filename))
    {
        yError("map file not found, closing");
        return false;
    }
    m_current_map.enlargeObstacles(6);
    m_iMap->store_map(m_current_map);
        
    return true;
}

void PlannerThread :: threadRelease()
{
    if (m_ptf.isValid()) m_ptf.close();
    if (m_pLas.isValid()) m_pLas.close();
    m_port_localization_input.interrupt();
    m_port_localization_input.close();
    m_port_map_output.interrupt();
    m_port_map_output.close();
    m_port_status_input.interrupt();
    m_port_status_input.close();
    m_port_status_output.interrupt();
    m_port_status_output.close();
    m_port_commands_output.interrupt();
    m_port_commands_output.close();
    m_port_yarpview_target_input.interrupt();
    m_port_yarpview_target_input.close();
    m_port_yarpview_target_output.interrupt();
    m_port_yarpview_target_output.close();
}


