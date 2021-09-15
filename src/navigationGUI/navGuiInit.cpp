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

YARP_LOG_COMPONENT(NAVIGATION_GUI_INIT, "navigation.navigationGui.init")

NavGuiThread::NavGuiThread(double _period, ResourceFinder &_rf) :
        PeriodicThread(_period), m_rf(_rf)
{
    m_navigation_status          = navigation_status_idle;
    m_previous_navigation_status = navigation_status_idle;

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

    button4_l = button3_r + 20;
    button4_r = button4_l + button_w;
    button4_t = 0;
    button4_b = button4_t + button_h;

    i1_map = nullptr;
    i2_map_menu = nullptr;
    i3_map_menu_scan = nullptr;
    i4_map_with_path = nullptr;
}

bool NavGuiThread::threadInit()
{
    //read configuration parameters
    std::string debug_rf = m_rf.toString();

    Bottle general_group = m_rf.findGroup("NAVIGATIONGUI_GENERAL");
    if (general_group.isNull())
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing NAVIGATIONGUI_GENERAL group!";
        return false;
    }

    Bottle laser_group = m_rf.findGroup("LASER");
    if (laser_group.isNull())
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing LASER group!";
        return false;
    }

    Bottle update_data_group = m_rf.findGroup("UPDATE_DATA");
    if (update_data_group.isNull())
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing UPDATE_DATA group!";
        return false;
    }

    Bottle drawing_group = m_rf.findGroup("DRAWING");
    if (drawing_group.isNull())
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing DRAWING group!";
        return false;
    }
/*
    Bottle geometry_group = m_rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }

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
        yCError() << "Invalid/missing parameter in ROBOT_GEOMETRY group";
        return false;
    }
*/

    //open module ports
    bool ret = true;
    if (general_group.check("local"))
    {
        m_name = general_group.find("local").asString();
    }

    //remote ports
    if (general_group.check("remote_localization"))
    {
        m_remote_localization_port_name = general_group.find("remote_localization").asString();
    }
    if (general_group.check("remote_navigation"))
    {
        m_remote_navigation_port_name = general_group.find("remote_navigation").asString();
    }
    if (general_group.check("remote_map"))
    {
        m_remote_map_port_name = general_group.find("remote_map").asString();
    }
    if (laser_group.check("remote_laser"))
    {
        m_remote_laser_port_name = laser_group.find("remote_laser").asString();
    }
    if (general_group.check("draw_and_publish_period"))
    {
        m_imagemap_draw_and_send_period = general_group.find("draw_and_publish_period").asFloat64();
    }

    //remote devices
    if (general_group.check("localization_client"))
    {
        m_localization_client_device_name = general_group.find("localization_client").asString();
    }
    else
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing localization_client parameter, e.g. localization2DClient / localization2D_nwc_yarp";
        return false;
    }
    if (general_group.check("navigation_client"))
    {
        m_navigation_client_device_name = general_group.find("navigation_client").asString();
    }
    else
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing navigation_client parameter, e.g. navigation2DClient / navigation2D_nwc_yarp";
        return false;
    }
    if (general_group.check("map_client"))
    {
        m_map_client_device_name = general_group.find("map_client").asString();
    }
    else
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing map_client parameter, e.g. map2DClient / map2D_nwc_yarp";
        return false;
    }
    if (laser_group.check("laser_client"))
    {
        m_laser_client_device_name = laser_group.find("laser_client").asString();
    }
    else
    {
        yCError(NAVIGATION_GUI_INIT) << "Missing laser_client parameter, e.g. Rangefinder2DClient / rangefinder2D_nwc_yarp";
        return false;
    }

    ret &= m_port_map_output.open((m_name + "/map:o").c_str());
    ret &= m_port_yarpview_target_input.open((m_name + "/yarpviewTarget:i").c_str());
    if (ret == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open module ports";
        return false;
    }

    //update_data_group
    if (update_data_group.check("period_laser_data"))
    {
        m_period_update_laser_data = update_data_group.find("period_laser_data").asFloat64();
    }
    else {}
    if (update_data_group.check("period_enalarged_obstacles"))
    {
        m_period_update_enlarged_obstacles = update_data_group.find("period_enalarged_obstacles").asFloat64();
    }
    else {}
    if (update_data_group.check("period_estimated_poses"))
    {
        m_period_update_estimated_poses = update_data_group.find("period_estimated_poses").asFloat64();
    }
    else {}
    if (update_data_group.check("period_map_locations"))
    {
        m_period_update_map_locations = update_data_group.find("period_map_locations").asFloat64();
    }
    else {}
    if (update_data_group.check("period_global_map"))
    {
        m_period_update_global_map = update_data_group.find("period_global_map").asFloat64();
    }
    else {}

    //drawing_group
    if (drawing_group.check("enable_draw_all_locations"))
    {
        m_enable_draw_all_locations = drawing_group.find("enable_draw_all_locations").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_enlarged_scans"))
    {
        m_enable_draw_enlarged_scans = drawing_group.find("enable_draw_enlarged_scans").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_laser_scans"))
    {
        m_enable_draw_laser_scans = drawing_group.find("enable_draw_laser_scans").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_infos"))
    {
        m_enable_draw_infos = drawing_group.find("enable_draw_infos").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_global_path"))
    {
        m_enable_draw_global_path = drawing_group.find("enable_draw_global_path").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_local_path"))
    {
        m_enable_draw_local_path = drawing_group.find("enable_draw_local_path").asBool();
    }
    else {}
    if (drawing_group.check("enable_draw_particles_number"))
    {
        m_enable_estimated_particles = drawing_group.find("enable_draw_particles_number").asInt16();
    }
    else {}

    //localization
    Property loc_options;
    loc_options.put("device", m_localization_client_device_name);
    loc_options.put("local", m_name+"/localizationClient");
    loc_options.put("remote", m_remote_localization_port_name);
    if (m_pLoc.open(loc_options) == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_pLoc.isValid() == false || m_iLoc == 0)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to view localization interface";
        return false;
    }

    //open the map interface
    Property map_options;
    map_options.put("device", m_map_client_device_name);
    map_options.put("local", m_name+"/map2DClient");
    map_options.put("remote", m_remote_map_port_name);
    if (m_pMap.open(map_options) == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open map2DClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open map interface";
        return false;
    }

    //open the navigation interface
    Property nav_options;
    nav_options.put("device", m_navigation_client_device_name);
    nav_options.put("local", m_name + "/navigation2DClient");
    nav_options.put("navigation_server", m_remote_navigation_port_name);
    nav_options.put("map_locations_server", m_remote_map_port_name);
    nav_options.put("localization_server", m_remote_localization_port_name);
    if (m_pNav.open(nav_options) == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open navigation2DClient";
        return false;
    }
    m_pNav.view(m_iNav);
    if (m_iNav == 0)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open navigation interface";
        return false;
    }

    //open the laser interface
    /*Bottle laserBottle = m_rf.findGroup("LASER");
    if (laserBottle.isNull())
    {
        yCError("LASER group not found,closing");
        return false;
    }
    if (laserBottle.check("laser_port") == false)
    {
        yCError("laser_port param not found,closing");
        return false;
    }
    string laser_remote_port = laserBottle.find("laser_port").asString();*/

    Property las_options;
    las_options.put("device", m_laser_client_device_name);
    las_options.put("local", m_name+"/laser2DClient");
    las_options.put("remote", m_remote_laser_port_name);
    if (m_pLas.open(las_options) == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Unable to open laser driver";
        return false;
    }
    m_pLas.view(m_iLaser);
    if (m_iLaser == 0)
    {
        yCWarning(NAVIGATION_GUI_INIT) << "Laser interface not available";
    }
    else
    {
        if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
        {
            yCError(NAVIGATION_GUI_INIT) << "Unable to obtain laser scan limits";
            return false;
        }
        m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);
    }

    //Get the maps
    if (readMaps() == false)
    {
        yCError(NAVIGATION_GUI_INIT) << "Error while reading maps";
    }

    return true;
}

void NavGuiThread:: threadRelease()
{
    std::lock_guard<std::mutex> lock(m_guithread_mutex);

    //beware: in order to prevent segfault (memory unloaded by the dll), the following vector
    //needs to be cleared BEFORE closing the device drivers with m_pLoc.close() m_pNav.close()
    m_estimated_poses.clear();

    if (m_ptf.isValid())  m_ptf.close(); 
    if (m_pLoc.isValid()) m_pLoc.close(); 
    if (m_pLas.isValid()) m_pLas.close(); 
    if (m_pMap.isValid()) m_pMap.close(); 
    if (m_pNav.isValid()) m_pNav.close();

    m_iLaser = nullptr;
    m_iMap = nullptr;
    m_iLoc = nullptr;
    m_iNav = nullptr;

    m_port_map_output.interrupt();
    m_port_map_output.close();
    m_port_yarpview_target_input.interrupt();
    m_port_yarpview_target_input.close();

    cvReleaseImage(&i1_map);
    cvReleaseImage(&i2_map_menu);
    cvReleaseImage(&i3_map_menu_scan);
    cvReleaseImage(&i4_map_with_path);
}


