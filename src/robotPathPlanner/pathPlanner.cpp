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
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <cv.h>
#include <highgui.h> 

#include "pathPlanner.h"
#include "pathPlannerHelpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

void PlannerThread::readTargetFromYarpView()
{
    yarp::os::Bottle *gui_targ = m_port_yarpview_target_input.read(false);
    if (gui_targ)
    {
        if (gui_targ->size() == 2)
        {
            MapGrid2D::XYCell c_start;
            c_start.x = (*gui_targ).get(0).asInt();
            c_start.y = (*gui_targ).get(1).asInt();
            yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(c_start));
            yInfo("selected point is located at (%6.3f, %6.3f)", v[0], v[1]);
            yarp::os::Bottle& out = m_port_yarpview_target_output.prepare();
            out.clear();
            out.addString("gotoAbs");
            out.addDouble(v[0]);
            out.addDouble(v[1]);
            m_port_yarpview_target_output.write();
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
            yarp::os::Bottle& out = m_port_yarpview_target_output.prepare();
            out.clear();
            out.addString("gotoAbs");
            out.addDouble(v[0]);
            out.addDouble(v[1]);
            out.addDouble(angle);
            m_port_yarpview_target_output.write();
        }
        else
        {
            yError() << "Received data with an invalid format.";
        }
    }
}

bool  PlannerThread::updateLocations()
{
    std::vector<yarp::os::ConstString> all_locations;
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

bool  PlannerThread::readLocalizationData()
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

    if (m_localization_data.map_id != m_current_map.getMapName())
    {
        yWarning() << "Current map name ("<<m_current_map.getMapName()<<") != m_localization_data.map_id ("<< m_localization_data.map_id <<")";
        yInfo() << "Asking the map '"<< m_localization_data.map_id << "' to the MAP server";
        bool map_get_succesfull = this->m_iMap->get_map(m_localization_data.map_id, m_current_map);
        if (map_get_succesfull)
        {
            m_temporary_obstacles_map = m_current_map;
            yInfo() << "Map '" << m_localization_data.map_id << "' succesfully obtained from server";
            m_current_map.enlargeObstacles(m_robot_radius);
            m_augmented_map = m_current_map;
            yDebug() << "Obstacles enlargement performed ("<<m_robot_radius<<"m)";
            updateLocations();
        }
        else
        {
            yError() << "Unable to get map '" << m_localization_data.map_id << "' from map server";
            std::vector<string> names_vector;
            m_iMap->get_map_names(names_vector);
            string names = "Known maps are:" ;
            for (auto it = names_vector.begin(); it != names_vector.end(); it++)
            {
                names = names + " " + (*it);
            }
            yInfo() << names;
            yarp::os::Time::delay(1.0);
            return true; //consider changing this to false
        }
    }

    return true;
}

bool  PlannerThread::readInnerNavigationStatus()
{
    static double last_print_time = 0;

    //read the internal navigation status
    Bottle cmd1, ans1;
    cmd1.addString("get");
    cmd1.addString("navigation_status");
    m_port_commands_output.write(cmd1, ans1);
    string s = ans1.get(0).toString().c_str();
    m_inner_status = pathPlannerHelpers::string2status(s);
    if (m_inner_status == navigation_status_error)
    {
        if (yarp::os::Time::now() - last_print_time > 1.0)
        {
            yError() << "Inner status = error"; 
            yarp::os::Time::delay(1.0);
            last_print_time = yarp::os::Time::now();
        }
        return false;
    }
    return true;
}

void  PlannerThread::readLaserData()
{
    std::vector<LaserMeasurementData> scan;
    bool ret = m_iLaser->getLaserMeasurement(scan);

    if (ret)
    {
        m_laser_map_cells.clear();
        unsigned int scansize = scan.size();
        for (unsigned int i = 0; i<scansize; i++)
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

    //transform the laser measurment in a temporary map
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

void PlannerThread::draw_map()
{
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.28, 0.28);
    static CvScalar red_color   = cvScalar(200, 80, 80);
    static CvScalar green_color = cvScalar(80, 200, 80);
    static CvScalar blue_color  = cvScalar(0, 0, 200);
    static CvScalar azure_color = cvScalar(80, 80, 200);
    MapGrid2D::XYCell start = m_current_map.world2Cell(MapGrid2D::XYWorld(m_localization_data.x, m_localization_data.y));
    MapGrid2D::XYCell final_goal = m_current_map.world2Cell(yarp::dev::MapGrid2D::XYWorld(m_final_goal.x, m_final_goal.y));

    yarp::sig::ImageOf<yarp::sig::PixelRgb> map_image;
    m_current_map.getMapImage(map_image);
    static IplImage*   processed_map_with_scan = 0;
    prepare_image(processed_map_with_scan,(const IplImage*) map_image.getIplImage());
    if (m_laser_timeout_counter<TIMEOUT_MAX)
    {
        if (m_enable_draw_enlarged_scans)
        {
            drawLaserMap(processed_map_with_scan, m_temporary_obstacles_map, azure_color);
        }
        if (m_enable_draw_laser_scans)
        {
            drawLaserScan(processed_map_with_scan, m_laser_map_cells, blue_color);
        }
    }

    //draw goal
    switch (m_planner_status)
    {
        case navigation_status_preparing_before_move:
        case navigation_status_moving:
        case navigation_status_waiting_obstacle:
        case navigation_status_aborted:
        case navigation_status_failing:
        case navigation_status_paused:
        case navigation_status_thinking:
            drawGoal(processed_map_with_scan, final_goal, m_final_goal.theta* DEG2RAD, red_color);
        break;
        case navigation_status_goal_reached:
            drawGoal(processed_map_with_scan, final_goal, m_final_goal.theta* DEG2RAD, green_color);
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
            drawGoal(processed_map_with_scan, m_current_map.world2Cell(MapGrid2D::XYWorld(m_locations_list[i].x, m_locations_list[i].y)), m_locations_list[i].theta* DEG2RAD, blue_color);
        }
    }

    drawCurrentPosition(processed_map_with_scan, start, m_localization_data.theta* DEG2RAD, azure_color);
#define DRAW_INFO
#ifdef DRAW_INFO
    MapGrid2D::XYWorld w_x_axis; w_x_axis.x = 2; w_x_axis.y = 0;
    MapGrid2D::XYWorld w_y_axis; w_y_axis.x = 0; w_y_axis.y = 2;
    MapGrid2D::XYWorld w_orig; w_orig.x = 0; w_orig.y = 0;
    MapGrid2D::XYCell x_axis = m_current_map.world2Cell(w_x_axis);
    MapGrid2D::XYCell y_axis = m_current_map.world2Cell(w_y_axis);
    MapGrid2D::XYCell orig = m_current_map.world2Cell(w_orig);
    drawInfo(processed_map_with_scan, start, orig, x_axis, y_axis, m_localization_data, font, blue_color);
#endif
    static IplImage* map_with_path = 0;
    prepare_image(map_with_path,processed_map_with_scan);

    CvScalar color = cvScalar(0, 200, 0);
    CvScalar color2 = cvScalar(0, 200, 100);

    if (m_planner_status != navigation_status_idle &&
        m_planner_status != navigation_status_goal_reached &&
        m_planner_status != navigation_status_aborted &&
        m_planner_status != navigation_status_error &&
        m_planner_status != navigation_status_failing)
    {
#ifdef DRAW_BOTH_PATHS
        drawPath(map_with_path, start, computed_path, color);
        drawPath(map_with_path, start, computed_simplified_path, color2);
#else
        MapGrid2D::XYCell current_path;
        if (getCurrentWaypoint(current_path))
        {
            drawPath(map_with_path, start, current_path, *m_current_path, color);
        }
#endif
    }

    static IplImage* map_with_location = 0;
    prepare_image(map_with_location,map_with_path);

    sendToPort(&m_port_map_output, map_with_location);
}

void PlannerThread::run()
{
    m_mutex.wait();
    //double check1 = yarp::os::Time::now();
    readTargetFromYarpView();
    readLocalizationData();
    readLaserData();
    //double check2 = yarp::os::Time::now();
    //yDebug() << check2-check1;
    if (readInnerNavigationStatus() == false)
    {
        m_planner_status = navigation_status_error;
        //yError() << "Error status";
        //yarp::os::Time::delay(1.0);
        //return;
    }

    /////////////////////////////
    // the finite-state-machine
    /////////////////////////////
    int path_size = m_current_path->size();
    if (m_planner_status == navigation_status_moving)
    {
        if (m_inner_status == navigation_status_goal_reached)
        {
            yInfo ("waypoint reached");
            if (path_size == 0)
            {
                //navigation is complete
                yInfo("navigation complete");
                m_planner_status = navigation_status_goal_reached;
            }
            else if (path_size == 1)
            {
                //remove the current waypoint, just reached
                m_current_path->pop();
                //send the next waypoint
                yInfo("sending the last waypoint");
                {
                    //send the tolerance to the inner controller
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("linear_tol");
                    cmd.addDouble(m_goal_tolerance_lin);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd2, ans2;
                    cmd2.addString("set");
                    cmd2.addString("angular_tol");
                    cmd2.addDouble(m_goal_tolerance_ang);
                    m_port_commands_output.write(cmd2, ans2);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("min_lin_speed");
                    cmd.addDouble(m_goal_min_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("max_lin_speed");
                    cmd.addDouble(m_goal_max_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("min_ang_speed");
                    cmd.addDouble(m_goal_min_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("max_ang_speed");
                    cmd.addDouble(m_goal_max_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("ang_speed_gain");
                    cmd.addDouble(m_goal_ang_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("lin_speed_gain");
                    cmd.addDouble(m_goal_lin_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                sendWaypoint();
            }
            else
            {
                //remove the current waypoint, just reached
                m_current_path->pop();
                //send the next waypoint
                yInfo("sending the next waypoint");
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("min_lin_speed");
                    cmd.addDouble(m_waypoint_min_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("max_lin_speed");
                    cmd.addDouble(m_waypoint_max_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("min_ang_speed");
                    cmd.addDouble(m_waypoint_min_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("max_ang_speed");
                    cmd.addDouble(m_waypoint_max_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("ang_speed_gain");
                    cmd.addDouble(m_waypoint_ang_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("lin_speed_gain");
                    cmd.addDouble(m_waypoint_lin_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                sendWaypoint();
            }
        }
        else if (m_inner_status == navigation_status_preparing_before_move)
        {
            //do nothing, just wait
        }
        else if (m_inner_status == navigation_status_moving)
        {
            //do nothing, just wait
        }
        else if (m_inner_status == navigation_status_waiting_obstacle)
        {
            //do nothing, just wait
        }
        else if (m_inner_status == navigation_status_failing)
        {
            if (m_enable_try_recovery)
            {
                //try to avoid obstacles
                yError ("unable to reach next waypoint, trying new solution");

                Bottle cmd, ans;
                cmd.addString("stop");
                m_port_commands_output.write(cmd, ans);
                update_obstacles_map(m_current_map, m_augmented_map);
                sendWaypoint();
            }
            else
            {
                //terminate navigation
                Bottle cmd, ans;
                cmd.addString("stop");
                m_port_commands_output.write(cmd, ans);
                m_planner_status = navigation_status_aborted;
                yError ("unable to reach next waypoint, aborting navigation");
            }
        }
        else if (m_inner_status == navigation_status_aborted)
        {
            //terminate navigation
            m_planner_status = navigation_status_aborted;
            yError ("unable to reach next waypoint, aborting navigation");
            //current_path.clear();
        }
        else if (m_inner_status == navigation_status_error)
        {
            yError("PathPlanner in error status");
            m_planner_status = navigation_status_error;
        }
        else if (m_inner_status == navigation_status_idle)
        {
            //send the first waypoint
            yInfo ("sending the first waypoint");
            //send the tolerance to the inner controller
            {
                Bottle cmd1, ans1;
                cmd1.addString("set"); 
                cmd1.addString("linear_tol");
                cmd1.addDouble(m_waypoint_tolerance_lin);
                m_port_commands_output.write(cmd1, ans1);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("angular_tol");
                cmd.addDouble(m_waypoint_tolerance_ang);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("max_lin_speed");
                cmd.addDouble(m_waypoint_max_lin_speed);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("max_ang_speed");
                cmd.addDouble(m_waypoint_max_ang_speed);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_lin_speed");
                cmd.addDouble(m_waypoint_min_lin_speed);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_ang_speed");
                cmd.addDouble(m_waypoint_min_ang_speed);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set");
                cmd.addString("ang_speed_gain");
                cmd.addDouble(m_waypoint_ang_gain);
                m_port_commands_output.write(cmd, ans);
            }
            {
                Bottle cmd, ans;
                cmd.addString("set");
                cmd.addString("lin_speed_gain");
                cmd.addDouble(m_waypoint_lin_gain);
                m_port_commands_output.write(cmd, ans);
            }
            sendWaypoint();
        }
        else
        {
            yError("unrecognized inner status: %d", m_inner_status);
        }
    }
    else if (m_planner_status == navigation_status_goal_reached)
    {
        //do nothing, just wait
    }
    else if (m_planner_status == navigation_status_idle)
    {
        //do nothing, just wait
    }
    else if (m_planner_status == navigation_status_thinking)
    {
        //do nothing, just wait
    }
    else if (m_planner_status == navigation_status_aborted)
    {
        //do nothing, just wait
    }
    else if (m_planner_status == navigation_status_failing)
    {
        //do nothing, just wait.
        //this status should be not reached by the high-level planner.
    }
    else if (m_planner_status == navigation_status_error)
    {
        if (m_inner_status != navigation_status_error)
        {
            m_planner_status = navigation_status_aborted;
        }
    }
    else
    {
        //unknown status
        yError("unknown status:%d", m_planner_status);
        m_planner_status = navigation_status_error;
    }

    //broadcast the planner status
    if (m_port_status_output.getOutputCount()>0)
    {
        string s = pathPlannerHelpers::getStatusAsString(m_planner_status);
        Bottle &b = m_port_status_output.prepare();
        b.clear();
        b.addString(s.c_str());
        m_port_status_output.write();
    }
    
    static double last_drawn = yarp::os::Time::now();
    double elapsed_time = yarp::os::Time::now() - last_drawn;
    if ( elapsed_time > m_imagemap_refresh_time)
    {
        //double check3 = yarp::os::Time::now();
        draw_map();
        //double check4 = yarp::os::Time::now();
        //yDebug() << check4-check3;
        last_drawn = yarp::os::Time::now();
    }
    
    m_mutex.post();
}

bool PlannerThread::getCurrentWaypoint(yarp::dev::MapGrid2D::XYCell &c) const
{
    if (m_current_path == NULL)
    {
        yError() << "PlannerThread::getCurrentWaypoint() m_current_path is NULL";
        return false;
    }
    if (m_current_path->size() == 0)
    {
        yError() << "PlannerThread::getCurrentWaypoint() m_current_path is empty (size==0)";
        return false;
    }
    c = m_current_path->front();
    return true;
}

void PlannerThread::sendWaypoint()
{
    int path_size = m_current_path->size();
    if (path_size==0)
    {
        yWarning ("Path queue is empty!");
        m_planner_status = navigation_status_idle;
        return;
    }
    //get the current waypoint from the list
    MapGrid2D::XYCell current_waypoint;
    if (getCurrentWaypoint(current_waypoint)==false)
    {
        yError("getCurrentWaypoint failed!");
        m_planner_status = navigation_status_idle;
        return;
    }

    //send the waypoint to the inner controller
    Bottle cmd1, ans1;
    cmd1.addString("gotoAbs");
    yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(current_waypoint));
    cmd1.addDouble(v[0]);
    cmd1.addDouble(v[1]);
    if (path_size == 1 && std::isnan(m_final_goal.theta) == false)
    {
        //add the orientation to the last waypoint
        cmd1.addDouble(m_final_goal.theta);
    }
    yDebug ("sending command: %s", cmd1.toString().c_str());
    m_port_commands_output.write(cmd1, ans1);
    //yDebug ("received answer: %s", ans1.toString().c_str());

    Bottle cmd2, ans2;
    cmd2.addString("get");
    cmd2.addString("navigation_status");
    //yDebug ("sending command: %s", cmd2.toString().c_str());
    m_port_commands_output.write(cmd2, ans2);
    //yDebug ("received answer: %s", ans2.toString().c_str());
    m_inner_status = pathPlannerHelpers::string2status(ans2.toString().c_str());
}

bool PlannerThread::startPath()
{
    yarp::math::Vec2D<double> start_vec;
    yarp::math::Vec2D<double> goal_vec;
    start_vec.x= m_localization_data.x;
    start_vec.y= m_localization_data.y;
    goal_vec.x = m_sequence_of_goals.front().x;
    goal_vec.y = m_sequence_of_goals.front().y;
    if (m_current_map.isInsideMap(start_vec) == false)
    {
        yError() << "PlannerThread::startPath() current robot location (" << start_vec.toString() << ")is not inside map" << m_current_map.getMapName();
        return false;
    }
    if (m_current_map.isInsideMap(goal_vec) == false)
    {
        yError() << "PlannerThread::startPath() requested goal (" << goal_vec.toString() << ") is not inside map" << m_current_map.getMapName();
        return false;
    }
    MapGrid2D::XYCell goal = m_current_map.world2Cell(goal_vec);
    MapGrid2D::XYCell start = m_current_map.world2Cell(start_vec);
#ifdef DEBUG_WITH_CELLS
    start.x = 150;//&&&&&
    start.y = 150;//&&&&&
#endif
    double t1 = yarp::os::Time::now();
    //clear the memory 
    std::queue<MapGrid2D::XYCell> empty;
    std::swap(m_computed_path, empty);
    std::queue<MapGrid2D::XYCell> empty2;
    std::swap(m_computed_simplified_path, empty2);
    m_planner_status = navigation_status_thinking;

    //search for a path
    bool b = findPath(m_current_map, start, goal, m_computed_path);
    if (!b)
    {
        yError ("path not found");
        m_planner_status = navigation_status_aborted;
        return false;
    }
    double t2 = yarp::os::Time::now();

    //search for an simpler path (waypoint optimization)
    simplifyPath(m_current_map, m_computed_path, m_computed_simplified_path);
    yInfo("path size:%d simplified path size:%d time: %.2f", m_computed_path.size(), m_computed_simplified_path.size(), t2 - t1);

    //choose the path to use
    if (m_use_optimized_path)
    {
        m_current_path = &m_computed_simplified_path;
    }
    else
    {
        m_current_path = &m_computed_path;
    }

    //just set the status to moving, do not set position commands.
    //The wayypoint ist set in the main 'run' loop.
    m_planner_status = navigation_status_moving;
    return true;
}

