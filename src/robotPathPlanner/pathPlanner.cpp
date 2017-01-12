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
            MapGrid2D::XYCell c_start;
            MapGrid2D::XYCell c_end;
            c_start.x = (*gui_targ).get(0).asInt();
            c_start.y = (*gui_targ).get(1).asInt();
            c_end.x = (*gui_targ).get(2).asInt();
            c_end.y = (*gui_targ).get(3).asInt();
            double angle = atan2(c_end.x - c_start.x, c_end.y - c_start.y) * 180.0 / M_PI;
            yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(c_start));
            yInfo("selected point is located at (%6.3f, %6.3f), angle:", v[0], v[1], angle);
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

void  PlannerThread::readLocalizationData()
{
    if (m_use_localization_from_port)
    {
        yarp::sig::Vector *loc = m_port_localization_input.read(false);
        if (loc)
        {
            m_localization_data.x = loc->data()[0];
            m_localization_data.y = loc->data()[1];
            m_localization_data.theta = loc->data()[2];
            m_localization_data.map_id = m_current_map.m_map_name;
            m_loc_timeout_counter = 0;
        }
        else
        {
            m_loc_timeout_counter++;
            if (m_loc_timeout_counter>TIMEOUT_MAX) m_loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else if (m_use_localization_from_tf)
    {
        yarp::sig::Vector iv;
        yarp::sig::Vector pose;
        iv.resize(6, 0.0);
        pose.resize(6, 0.0);
        bool r = m_iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);
        if (r)
        {
            //data is formatted as follows: x, y, angle (in degrees)
            m_localization_data.map_id = m_current_map.m_map_name;
            m_localization_data.x = pose[0];
            m_localization_data.y = pose[1];
            m_localization_data.theta = pose[5] * RAD2DEG;
            m_loc_timeout_counter = 0;
        }
        else
        {
            m_loc_timeout_counter++;
            if (m_loc_timeout_counter > TIMEOUT_MAX) m_loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else
    {
        yWarning() << "Localization disabled";
    }
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
            yarp::sig::Vector v(2);
            double ss = cos(m_localization_data.theta * DEG2RAD);
            double cs = sin(m_localization_data.theta * DEG2RAD);
            v[0] = las_x*cs - las_y*ss + m_localization_data.x;
            v[1] = las_x*ss + las_y*cs + m_localization_data.y;
            m_laser_map_cells.push_back(m_current_map.world2Cell(v));
        }
        m_laser_timeout_counter = 0;
    }
    else
    {
        m_laser_timeout_counter++;
    }
}

void PlannerThread::draw_map()
{
    static CvScalar blue_color = cvScalar(0, 0, 200);
    static CvScalar blue_color2 = cvScalar(80, 80, 200);
    MapGrid2D::XYCell start = m_current_map.world2Cell(MapGrid2D::XYWorld(m_localization_data.x, m_localization_data.y));

    yarp::sig::ImageOf<yarp::sig::PixelRgb> map_image;
    m_current_map.getMapImage(map_image);
    static IplImage*   processed_map_with_scan = 0;
    if (processed_map_with_scan == 0) processed_map_with_scan = cvCloneImage((const IplImage*)map_image.getIplImage());
    cvCopyImage(map_image.getIplImage(), processed_map_with_scan);
    if (m_laser_timeout_counter<TIMEOUT_MAX)
    {
        drawLaserScan(processed_map_with_scan, m_laser_map_cells, blue_color);
        //map.enlargeScan(laser_map_cell,6);
        //map.drawLaserScan(map.processed_map_with_scan,laser_map_cell,blue_color2);
    }

    drawCurrentPosition(processed_map_with_scan, start, m_localization_data.theta* DEG2RAD, blue_color);
    static IplImage* map_with_path = 0;
    if (map_with_path == 0) map_with_path = cvCloneImage(processed_map_with_scan);
    else cvCopyImage(processed_map_with_scan, map_with_path);

    CvScalar color = cvScalar(0, 200, 0);
    CvScalar color2 = cvScalar(0, 200, 100);

    if (m_planner_status != navigation_status_idle && m_planner_status != navigation_status_goal_reached)
    {
#ifdef DRAW_BOTH_PATHS
        drawPath(map_with_path, start, computed_path, color);
        drawPath(map_with_path, start, computed_simplified_path, color2);
#else
        drawPath(map_with_path, start, m_current_waypoint, *m_current_path, color);
#endif
    }

    static IplImage* map_with_location = 0;
    if (map_with_location == 0) map_with_location = cvCloneImage(map_with_path);
    else cvCopyImage(map_with_path, map_with_location);

    sendToPort(&m_port_map_output, map_with_location);
}

void PlannerThread::run()
{
    m_mutex.wait();
    readTargetFromYarpView();
    readLocalizationData();
    readLaserData();

    //read the internal navigation status
    Bottle cmd1, ans1;
    cmd1.addString("get"); 
    cmd1.addString("navigation_status");
    m_port_commands_output.write(cmd1, ans1);
    string s = ans1.get(0).toString().c_str();
    m_inner_status = pathPlannerHelpers::string2status(s);

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
                 //send the next waypoint
                yInfo("sending the last waypoint");

                //send the tolerance to the inner controller
                Bottle cmd1, ans1;
                cmd1.addString("set"); 
                cmd1.addString("linear_tol");
                cmd1.addDouble(m_goal_tolerance_lin);
                m_port_commands_output.write(cmd1, ans1);

                Bottle cmd2, ans2;
                cmd2.addString("set"); 
                cmd2.addString("angular_tol");
                cmd2.addDouble(m_goal_tolerance_ang);
                m_port_commands_output.write(cmd2, ans2);

                //last waypoint ha minimum linear speed = 0
                Bottle cmd3, ans3;
                cmd3.addString("set"); 
                cmd3.addString("min_lin_speed");
                cmd3.addDouble(0.0);
                m_port_commands_output.write(cmd3, ans3);

                sendWaypoint();
            }
            else
            {
                //send the next waypoint
                yInfo("sending the next waypoint");
                sendWaypoint();
                {
                    //here I send min_lin_speed = max_lin_speed to have constant velocity
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("min_lin_speed");
                    cmd.addDouble(m_max_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("max_lin_speed");
                    cmd.addDouble(m_max_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
            }
        }
        else if (m_inner_status == navigation_status_moving)
        {
            //do nothing, just wait
        }
        else if (m_inner_status == navigation_status_waiting_obstacle)
        {
            //do nothing, just wait
        }
        else if (m_inner_status == navigation_status_aborted)
        {
            //terminate navigation
            m_planner_status = navigation_status_aborted;
            yError ("unable to reach next waypoint, aborting navigation");
            //current_path.clear();
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
                cmd.addDouble(m_max_lin_speed);
                m_port_commands_output.write(cmd, ans);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("max_ang_speed");
                cmd.addDouble(m_max_ang_speed);
                m_port_commands_output.write(cmd, ans);
            }

            {
                //here I send min_lin_speed = max_lin_speed to have constant velocity
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_lin_speed");
                cmd.addDouble(m_max_lin_speed);
                m_port_commands_output.write(cmd, ans);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_ang_speed");
                cmd.addDouble(m_min_ang_speed);
                m_port_commands_output.write(cmd, ans);
                sendWaypoint();
            }
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
    else
    {
        //unknown status
        yError("unknown status:%d", m_planner_status);
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
    
    draw_map();
    m_mutex.post();
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
    //get the next waypoint from the list
    m_current_waypoint = m_current_path->front();
    m_current_path->pop();
    //send the waypoint to the inner controller
    Bottle cmd1, ans1;
    cmd1.addString("gotoAbs");
    yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(m_current_waypoint));
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

void PlannerThread::startPath()
{
    yarp::math::Vec2D<double> start_vec;
    yarp::math::Vec2D<double> goal_vec;
    start_vec.x= m_localization_data.x;
    start_vec.y= m_localization_data.y;
    goal_vec.x = m_sequence_of_goals.front().x;
    goal_vec.y = m_sequence_of_goals.front().y;
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
        return;
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
}

