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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/INavigation2D.h>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <cv.h>
#include <highgui.h> 

#include "navGui.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

void NavGuiThread::readTargetFromYarpView()
{
    yarp::os::Bottle *gui_targ = m_port_yarpview_target_input.read(false);
    if (gui_targ)
    {
        if (gui_targ->size() == 2)
        {
            MapGrid2D::XYCell c_end_gui;
            c_end_gui.x = (*gui_targ).get(0).asInt();
            c_end_gui.y = (*gui_targ).get(1).asInt();
            yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(c_end_gui));
            yInfo("selected point is located at (%6.3f, %6.3f)", v[0], v[1]);
            Map2DLocation loc;
            loc.map_id = m_localization_data.map_id;
            loc.x = v[0];
            loc.y = v[1];
            loc.theta = 0; //@@@@TO BE IMPROVED
            m_iNav->gotoTargetByAbsoluteLocation(loc);
        }
        else if (gui_targ->size() == 4)
        {
            MapGrid2D::XYCell c_start_gui;
            MapGrid2D::XYCell c_end_gui;
            MapGrid2D::XYWorld c_start_world;
            MapGrid2D::XYWorld c_end_world;
            c_start_gui.x = (*gui_targ).get(0).asInt();
            c_start_gui.y = (*gui_targ).get(1).asInt();
            c_end_gui.x = (*gui_targ).get(2).asInt();
            c_end_gui.y = (*gui_targ).get(3).asInt();
            c_start_world = (m_current_map.cell2World(c_start_gui));
            c_end_world = (m_current_map.cell2World(c_end_gui));
            double angle = atan2(c_end_world.y - c_start_world.y, c_end_world.x - c_start_world.x) * 180.0 / M_PI;
            yarp::sig::Vector v = static_cast<yarp::sig::Vector>(c_start_world);
            yInfo("selected point is located at (%6.3f, %6.3f), angle: %6.3f", v[0], v[1], angle);
            Map2DLocation loc;
            loc.map_id = m_localization_data.map_id;
            loc.x = v[0];
            loc.y = v[1];
            loc.theta = angle;
            m_iNav->gotoTargetByAbsoluteLocation(loc);
        }
        else
        {
            yError() << "Received data with an invalid format.";
        }
    }
}

bool  NavGuiThread::updateLocations()
{
    std::vector<std::string> all_locations;
    m_iMap->getLocationsList(all_locations);
    Map2DLocation tmp_loc;
    m_locations_list.clear();
    for (size_t i=0; i<all_locations.size(); i++)
    {
        m_iMap->getLocation(all_locations[i],tmp_loc);
        m_locations_list.push_back(tmp_loc);
    }
    return true;
}

bool  NavGuiThread::readMaps()
{
    m_iNav->getCurrentNavigationMap(yarp::dev::global_map, m_current_map);
}

bool  NavGuiThread::readLocalizationData()
{
    bool ret = m_iLoc->getCurrentPosition(m_localization_data);
    if (ret)
    {
        m_loc_timeout_counter = 0;
    }
    else
    {
        m_loc_timeout_counter++;
        if (m_loc_timeout_counter>TIMEOUT_MAX) m_loc_timeout_counter = TIMEOUT_MAX;
        return false;
    }

    return true;
}

bool  NavGuiThread::readNavigationStatus(bool& changed)
{
    static double last_print_time = 0;

    //read the navigation status
    m_iNav->getNavigationStatus(m_navigation_status);
    if (m_navigation_status != m_previous_navigation_status)
    {
        changed = true;
    }
    else
    {
        changed = false;
    }
    m_previous_navigation_status = m_navigation_status;

    if (m_navigation_status == navigation_status_error)
    {
        if (yarp::os::Time::now() - last_print_time > 1.0)
        {
            yError() << "Navigation status = error"; 
            last_print_time = yarp::os::Time::now();
        }
        return false;
    }
    return true;
}

void  NavGuiThread::readLaserData()
{
    std::vector<LaserMeasurementData> scan;
    bool ret = m_iLaser->getLaserMeasurement(scan);

    if (ret)
    {
        m_laser_map_cells.clear();
        size_t scansize = scan.size();
        for (size_t i = 0; i<scansize; i++)
        {
            double las_x = 0;
            double las_y = 0;
            scan[i].get_cartesian(las_x, las_y);
            //performs a rotation from the robot to the world reference frame
            MapGrid2D::XYWorld world;
            double ss = sin(m_localization_data.theta * DEG2RAD);
            double cs = cos(m_localization_data.theta * DEG2RAD);
            world.x = las_x*cs - las_y*ss + m_localization_data.x;
            world.y = las_x*ss + las_y*cs + m_localization_data.y;
        //    if (!std::isinf(world.x) &&  !std::isinf(world.y))
            if (std::isfinite(world.x) && std::isfinite(world.y))
               { m_laser_map_cells.push_back(m_current_map.world2Cell(world));}
        }
        m_laser_timeout_counter = 0;
    }
    else
    {
        m_laser_timeout_counter++;
    }

    //transform the laser measurement in a temporary map
    for (size_t y=0; y< m_temporary_obstacles_map.height(); y++)
        for (size_t x=0; x< m_temporary_obstacles_map.width(); x++)
             m_temporary_obstacles_map.setMapFlag(MapGrid2D::XYCell(x,y),MapGrid2D::MAP_CELL_FREE);
    for (size_t i=0; i< m_laser_map_cells.size(); i++)
    {
        m_temporary_obstacles_map.setMapFlag(m_laser_map_cells[i],MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE);
    }
    //enlarge the laser scanner data
    m_temporary_obstacles_map.enlargeObstacles(m_robot_radius);
}

bool prepare_image(IplImage* & image_to_be_prepared, const IplImage* template_image)
{
    if (template_image == 0)
    {
        yError() << "PlannerThread::draw_map cannot copy an empty image!";
        return false;
    }
    if (image_to_be_prepared == 0) 
    {
        image_to_be_prepared = cvCloneImage(template_image);
    }
    else if (image_to_be_prepared->width != template_image->width ||
             image_to_be_prepared->height != template_image->height)
    {
        cvResize(template_image, image_to_be_prepared);
        cvCopy(template_image, image_to_be_prepared);
    }
    else
    {
        cvCopy(template_image, image_to_be_prepared);
    }
    return true;
}

void NavGuiThread::draw_map()
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.28, 0.28);
    static CvScalar red_color   = cvScalar(200, 80, 80);
    static CvScalar green_color = cvScalar(80, 200, 80);
    static CvScalar blue_color  = cvScalar(0, 0, 200);
    static CvScalar azure_color = cvScalar(80, 80, 200);
    MapGrid2D::XYCell start = m_current_map.world2Cell(MapGrid2D::XYWorld(m_localization_data.x, m_localization_data.y));
    MapGrid2D::XYCell final_goal = m_current_map.world2Cell(yarp::dev::MapGrid2D::XYWorld(m_curr_goal.x, m_curr_goal.y));

    yarp::sig::ImageOf<yarp::sig::PixelRgb> map_image;
    m_current_map.getMapImage(map_image);
    static IplImage*   processed_map_with_scan = 0;
    prepare_image(processed_map_with_scan,(const IplImage*) map_image.getIplImage());
    if (m_laser_timeout_counter<TIMEOUT_MAX)
    {
        if (m_enable_draw_enlarged_scans)
        {
            map_utilites::drawLaserMap(processed_map_with_scan, m_temporary_obstacles_map, azure_color);
        }
        if (m_enable_draw_laser_scans)
        {
            map_utilites::drawLaserScan(processed_map_with_scan, m_laser_map_cells, blue_color);
        }
    }

    //draw goal
    switch (m_navigation_status)
    {
        case navigation_status_preparing_before_move:
        case navigation_status_moving:
        case navigation_status_waiting_obstacle:
        case navigation_status_aborted:
        case navigation_status_failing:
        case navigation_status_paused:
        case navigation_status_thinking:
            map_utilites::drawGoal(processed_map_with_scan, final_goal, m_curr_goal.theta* DEG2RAD, red_color);
        break;
        case navigation_status_goal_reached:
            map_utilites::drawGoal(processed_map_with_scan, final_goal, m_curr_goal.theta* DEG2RAD, green_color);
        break;
        case navigation_status_idle:
        default:
            //do nothing
        break;
    }

    if (m_enable_draw_all_locations)
    {
        for (size_t i=0; i<m_locations_list.size(); i++)
        {
            map_utilites::drawGoal(processed_map_with_scan, m_current_map.world2Cell(MapGrid2D::XYWorld(m_locations_list[i].x, m_locations_list[i].y)), m_locations_list[i].theta* DEG2RAD, blue_color);
        }
    }

    //draw Current Position
    map_utilites::drawCurrentPosition(processed_map_with_scan, start, m_localization_data.theta* DEG2RAD, azure_color);

    //draw Infos
#define DRAW_INFO
#ifdef DRAW_INFO
    MapGrid2D::XYWorld w_x_axis; w_x_axis.x = 2; w_x_axis.y = 0;
    MapGrid2D::XYWorld w_y_axis; w_y_axis.x = 0; w_y_axis.y = 2;
    MapGrid2D::XYWorld w_orig; w_orig.x = 0; w_orig.y = 0;
    MapGrid2D::XYCell x_axis = m_current_map.world2Cell(w_x_axis);
    MapGrid2D::XYCell y_axis = m_current_map.world2Cell(w_y_axis);
    MapGrid2D::XYCell orig = m_current_map.world2Cell(w_orig);
    map_utilites::drawInfo(processed_map_with_scan, start, orig, x_axis, y_axis, m_localization_data, font, blue_color);
#endif

    //draw path
    static IplImage* map_with_path = 0;
    prepare_image(map_with_path,processed_map_with_scan);

    CvScalar color = cvScalar(0, 200, 0);
    CvScalar color2 = cvScalar(0, 200, 100);

    if (m_navigation_status != navigation_status_idle &&
        m_navigation_status != navigation_status_goal_reached &&
        m_navigation_status != navigation_status_aborted &&
        m_navigation_status != navigation_status_error &&
        m_navigation_status != navigation_status_failing)
    {
        map_utilites::drawPath(map_with_path, start, m_current_path, *m_current_path, color);
    }

    //draw locations
    static IplImage* map_with_location = 0;
    prepare_image(map_with_location,map_with_path);

    map_utilites::sendToPort(&m_port_map_output, map_with_location);
}

void NavGuiThread::run()
{
    m_mutex.wait();
    //double check1 = yarp::os::Time::now();

    bool changed = false;
    readNavigationStatus(changed);


    readTargetFromYarpView();
    readLocalizationData();
    readLaserData();
    //double check2 = yarp::os::Time::now();
    //yDebug() << check2-check1;

    if (m_navigation_status == navigation_status_moving)
    {
        readWaypointsAndGoal();
    }
    else if (m_navigation_status == navigation_status_goal_reached)
    {
        //do nothing, just wait
    }
    else if (m_navigation_status == navigation_status_idle)
    {
        //do nothing, just wait
    }
    else if (m_navigation_status == navigation_status_thinking)
    {
        //do nothing, just wait
    }
    else if (m_navigation_status == navigation_status_aborted)
    {
        //do nothing, just wait
    }
    else if (m_navigation_status == navigation_status_failing)
    {
        //do nothing, just wait.
    }
    else if (m_navigation_status == navigation_status_error)
    {
        //do nothing, just wait.
    }
    else
    {
        //unknown status
        yError("unknown status:%d", m_navigation_status);
        m_navigation_status = navigation_status_error;
    }
    
    static double last_drawn = yarp::os::Time::now();
    double elapsed_time = yarp::os::Time::now() - last_drawn;
    if ( elapsed_time > m_imagemap_refresh_period)
    {
        //double check3 = yarp::os::Time::now();
        draw_map();
        //double check4 = yarp::os::Time::now();
        //yDebug() << check4-check3;
        last_drawn = yarp::os::Time::now();
    }
    
    m_mutex.post();
}

bool NavGuiThread::readWaypointsAndGoal()
{
    if (m_iNav)
    {
        m_iNav->getCurrentNavigationWaypoint(m_curr_waypoint);
        m_iNav->getAllNavigationWaypoints(m_all_waypoints);
        m_iNav->getAbsoluteLocationOfCurrentTarget(m_curr_goal);
    }
    return true;
}

void NavGuiThread::sendTargetFromYarpView()
{
}


