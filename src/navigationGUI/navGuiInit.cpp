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

#include "navGui.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

NavGuiThread::NavGuiThread(double _period, ResourceFinder &_rf) :
        PeriodicThread(_period), m_rf(_rf)
{
    m_navigation_status          = navigation_status_idle;
    m_previous_navigation_status = navigation_status_idle;
    m_loc_timeout_counter = 0;
    m_laser_timeout_counter = 0;
    m_nav_status_timeout_counter = 0;
    m_iLaser = 0;
    m_iLoc = 0;
    m_iNav = 0;
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    m_imagemap_refresh_period = 0.033;
    m_enable_draw_all_locations=true;
    m_enable_draw_enlarged_scans=true;
    m_enable_draw_laser_scans=true;
    m_enable_estimated_particles = 50;
    m_local_name_prefix = "/navigationGui";
    m_remote_localization = "/localizationServer";
    m_remote_map = "/mapServer";
    m_remote_laser = "/ikart/laser:o";
    m_remote_navigation = "/navigationServer";

    const int button_w = 70;
    const int button_h = 20;

    button1_l = 0;
    button1_r = button1_l+ button_w;
    button1_t = 0;
    button1_b = button1_t+ button_h;

    button2_l = button1_r + 20;
    button2_r = button2_l + button_w;
    button2_t = 0;
    button2_b = button2_t + button_h;

    button3_l = button2_r + 20;
    button3_r = button3_l + button_w;
    button3_t = 0;
    button3_b = button3_t + button_h;

    button3_status = button_status_goto;

    i1_map = nullptr;
    i2_map_menu = nullptr;
    i3_map_menu_scan = nullptr;
    i4_map_with_path = nullptr;
}

bool NavGuiThread::threadInit()
{
    //read configuration parameters
    std::string debug_rf = m_rf.toString();

    Bottle general_group = m_rf.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError() << "Missing GENERAL group!";
        return false;
    }

    Bottle laser_group = m_rf.findGroup("LASER");
    if (laser_group.isNull())
    {
        yError() << "Missing LASER group!";
        return false;
    }

//    if (general_group.check("publish_map_image_Hz"))       { m_imagemap_refresh_period = 1/general_group.find("publish_map_image_Hz").asDouble(); }
    if (general_group.check("publish_estimated_poses_Hz")) { m_period_draw_estimated_poses = 1/general_group.find("publish_estimated_poses_Hz").asDouble(); }

/*
    Bottle geometry_group = m_rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }

    bool ff = geometry_group.check("robot_radius");
    ff &= geometry_group.check("laser_pos_x");
    ff &= geometry_group.check("laser_pos_y");
    ff &= geometry_group.check("laser_pos_theta");

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
*/

    //open module ports
    bool ret = true;
    if (general_group.check("local"))
    {
        m_local_name_prefix = general_group.find("local").asString();
    }
    if (general_group.check("remote_localization"))
    {
        m_remote_localization = general_group.find("remote_localization").asString();
    }
    if (general_group.check("remote_map"))
    {
        m_remote_map = general_group.find("remote_map").asString();
    }
    if (laser_group.check("remote_laser"))
    {
        m_remote_laser = laser_group.find("remote_laser").asString();
    }
    ret &= m_port_map_output.open((m_local_name_prefix + "/map:o").c_str());
    ret &= m_port_yarpview_target_input.open((m_local_name_prefix + "/yarpviewTarget:i").c_str());
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    //localization
    Property loc_options;
    loc_options.put("device", "localization2DClient");
    loc_options.put("local", m_local_name_prefix+"/localizationClient");
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

    //open the map interface
    Property map_options;
    map_options.put("device", "map2DClient");
    map_options.put("local", m_local_name_prefix+"/map2DClient");
    map_options.put("remote", m_remote_map);
    if (m_pMap.open(map_options) == false)
    {
        yError() << "Unable to open map2DClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yError() << "Unable to open map interface";
        return false;
    }

    //open the navigation interface
    Property nav_options;
    nav_options.put("device", "navigation2DClient");
    nav_options.put("local", m_local_name_prefix + "/navigation2DClient");
    nav_options.put("navigation_server", m_remote_navigation);
    nav_options.put("map_locations_server", m_remote_map);
    nav_options.put("localization_server", m_remote_localization);
    if (m_pNav.open(nav_options) == false)
    {
        yError() << "Unable to open navigation2DClient";
        return false;
    }
    m_pNav.view(m_iNav);
    if (m_iNav == 0)
    {
        yError() << "Unable to open navigation interface";
        return false;
    }

    //open the laser interface
    /*Bottle laserBottle = m_rf.findGroup("LASER");
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
    string laser_remote_port = laserBottle.find("laser_port").asString();*/

    Property las_options;
    las_options.put("device", "Rangefinder2DClient");
    las_options.put("local", m_local_name_prefix+"/laser2DClient");
    las_options.put("remote", m_remote_laser);
    if (m_pLas.open(las_options) == false)
    {
        yError() << "Unable to open laser driver";
        return false;
    }
    m_pLas.view(m_iLaser);
    if (m_iLaser == 0)
    {
        yWarning() << "Laser interface not available";
    }
    else
    {
        if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
        {
            yError() << "Unable to obtain laser scan limits";
            return false;
        }
        m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);
    }

    //Get the maps
    readMaps();

    //update
    m_period_draw_laser = 0.3; //seconds;
    m_period_draw_enalarged_obstacles = 1.0; //seconds
    m_period_draw_estimated_poses = 1.0; //seconds

    return true;
}

void NavGuiThread:: threadRelease()
{
    if (m_pLoc.isValid()) m_pLoc.close();
    if (m_ptf.isValid()) m_ptf.close();
    if (m_pLas.isValid()) m_pLas.close();
    if (m_pNav.isValid()) m_pNav.close();
    m_port_map_output.interrupt();
    m_port_map_output.close();
    m_port_yarpview_target_input.interrupt();
    m_port_yarpview_target_input.close();
}


