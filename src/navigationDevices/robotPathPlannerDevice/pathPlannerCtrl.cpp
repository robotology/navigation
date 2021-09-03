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

#include <opencv2/imgproc/imgproc_c.h>

#include "pathPlannerCtrl.h"
#include "pathPlannerCtrlHelpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

YARP_LOG_COMPONENT(PATHPLAN_CTRL, "navigation.devices.robotPathPlanner.ctrl")

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

    if (m_localization_data.map_id != m_current_map.getMapName() ||
        m_force_map_reload == true)
    {
        if (m_force_map_reload)
        {
            yCInfo(PATHPLAN_CTRL) << "m_force_map_reload requested";
        }
        m_force_map_reload = false;
        yCWarning(PATHPLAN_CTRL) << "Current map name ("<<m_current_map.getMapName()<<") != m_localization_data.map_id ("<< m_localization_data.map_id <<")";
        yCInfo(PATHPLAN_CTRL) << "Asking the map '"<< m_localization_data.map_id << "' to the MAP server";
        bool b = reloadCurrentMap();
    
        yarp::os::Time::delay(1.0);
        if (b){return true;}
        else  { return true; //consider changing this to false
        }
    }

    return true;
}

bool  PlannerThread::setRobotRadius(double size)
{
    m_robot_radius = size;
    m_force_map_reload = true;
    return true;
}

bool  PlannerThread::getRobotRadius(double& size)
{
    size = m_robot_radius;
    return true;
}

bool  PlannerThread::internal_controller_t::readInnerNavigationStatus()
{
    static double last_print_time = 0;

    //read the internal navigation status
    NavigationStatusEnum inner_status;
    m_iInnerNav_ctrl->getNavigationStatus(inner_status);
    m_inner_status = inner_status;
    if (m_inner_status == navigation_status_error)
    {
        if (yarp::os::Time::now() - last_print_time > 1.0)
        {
            yCError(PATHPLAN_CTRL) << "Inner status = error"; 
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
        size_t scansize = scan.size();
        for (size_t i = 0; i<scansize; i++)
        {
            double las_x = 0;
            double las_y = 0;
            scan[i].get_cartesian(las_x, las_y);
            //performs a rotation from the robot to the world reference frame
            XYWorld world;
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
    //the purpose of the following copy of m_temporary_obstacles_map in temp_map is to minimize the duration of the critical section
    //protected by the mutex. Indeed enlargeObstacles() can take some time and should be outside the mutex.
    m_temporary_obstacles_map_mutex.lock();
    MapGrid2D temp_map = m_temporary_obstacles_map;
    m_temporary_obstacles_map_mutex.unlock();
    //here I am cleaning temp map from everything.
    for (size_t y=0; y< temp_map.height(); y++)
       for (size_t x=0; x< temp_map.width(); x++)
            temp_map.setMapFlag(XYCell(x,y),MapGrid2D::MAP_CELL_FREE);
    //Fill the temp map with laser scans
    for (size_t i=0; i< m_laser_map_cells.size(); i++)
    {
        temp_map.setMapFlag(m_laser_map_cells[i],MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE);
    }
    //enlarge the laser scans
    temp_map.enlargeObstacles(m_robot_radius);
    //m_temporary_obstacles_map is now filled only with MAP_CELL_FREE,MAP_CELL_TEMPORARY_OBSTACLE and MAP_CELL_ENLARGED_OBSTACLE
    m_temporary_obstacles_map_mutex.lock();
    m_temporary_obstacles_map = temp_map;
    m_temporary_obstacles_map_mutex.unlock();
}

bool prepare_image(IplImage* & image_to_be_prepared, const IplImage* template_image)
{
    if (template_image == 0)
    {
        yCError(PATHPLAN_CTRL) << "PlannerThread::draw_map cannot copy an empty image!";
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

void PlannerThread::run()
{
    double m_stats_time_curr = yarp::os::Time::now();
    if (m_stats_time_curr - m_stats_time_last > 5.0)
    {
        m_stats_time_last = m_stats_time_curr;
        bool err = false;
        if (m_laser_timeout_counter > TIMEOUT_MAX)
        {
            yCError(PATHPLAN_CTRL,"timeout, no laser data received!\n");
            err = true;
        }
        if (m_loc_timeout_counter > TIMEOUT_MAX)
        {
            yCError(PATHPLAN_CTRL, " timeout, no localization data received!\n");
            err = true;
        }
        if (m_inner_controller.m_inner_status_timeout_counter > TIMEOUT_MAX)
        {
            yCError(PATHPLAN_CTRL, "timeout, no status info received!\n");
            err = true;
        }
        if (err == false)
            yCInfo(PATHPLAN_CTRL) << "robotPathPlanner running, ALL ok. Navigation status:" << this->getNavigationStatusAsString();
    }
    
    m_mutex.wait();
    //double check1 = yarp::os::Time::now();
    readLocalizationData();
    readLaserData();
    //double check2 = yarp::os::Time::now();
    //yCDebug() << check2-check1;
    if (m_inner_controller.readInnerNavigationStatus() == false)
    {
        m_planner_status = navigation_status_error;
        //yCError(PATHPLAN_CTRL) << "Error status";
        //yarp::os::Time::delay(1.0);
        //return;
    }

    /////////////////////////////
    // the finite-state-machine
    /////////////////////////////
    switch (m_planner_status)
    {
        case navigation_status_moving:
        {
            if (m_inner_controller.m_inner_status == navigation_status_goal_reached)
            {
                if (m_current_path_iterator == m_current_path->end())
                {
                    //navigation is complete
                    yCInfo(PATHPLAN_CTRL, "goal reached, navigation complete");
                    m_planner_status = navigation_status_goal_reached;
                    m_final_goal_reached_at_timeX = yarp::os::Time::now();
                }
                else if (m_current_path_iterator == m_current_path->end()-1)
                {
                    yCInfo(PATHPLAN_CTRL, "waypoint reached");
                    //remove the current waypoint, just reached
                    m_current_path_iterator++;
                    m_remaining_path.pop_front();

                    //send the final waypoint
                    yCInfo(PATHPLAN_CTRL, "sending the last waypoint (final goal)");
                    {
                        //send the tolerance to the inner controller
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("linear_tol");
                        cmd.addFloat64(m_goal_tolerance_lin);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd2, ans2;
                        cmd2.addString("set");
                        cmd2.addString("angular_tol");
                        cmd2.addFloat64(m_goal_tolerance_ang);
                        m_port_commands_output.write(cmd2, ans2);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("min_lin_speed");
                        cmd.addFloat64(m_goal_min_lin_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("max_lin_speed");
                        cmd.addFloat64(m_goal_max_lin_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("min_ang_speed");
                        cmd.addFloat64(m_goal_min_ang_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("max_ang_speed");
                        cmd.addFloat64(m_goal_max_ang_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("ang_speed_gain");
                        cmd.addFloat64(m_goal_ang_gain);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("lin_speed_gain");
                        cmd.addFloat64(m_goal_lin_gain);
                        m_port_commands_output.write(cmd, ans);
                    }
                    sendFinalGoal();
                }
                else
                {
                    yCInfo(PATHPLAN_CTRL, "waypoint reached");
                    //remove the current waypoint, just reached
                    m_current_path_iterator++;
                    m_remaining_path.pop_front();

                    //send the next waypoint
                    yCInfo(PATHPLAN_CTRL, "sending the next waypoint");
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("min_lin_speed");
                        cmd.addFloat64(m_waypoint_min_lin_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("max_lin_speed");
                        cmd.addFloat64(m_waypoint_max_lin_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("min_ang_speed");
                        cmd.addFloat64(m_waypoint_min_ang_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("max_ang_speed");
                        cmd.addFloat64(m_waypoint_max_ang_speed);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("ang_speed_gain");
                        cmd.addFloat64(m_waypoint_ang_gain);
                        m_port_commands_output.write(cmd, ans);
                    }
                    {
                        Bottle cmd, ans;
                        cmd.addString("set");
                        cmd.addString("lin_speed_gain");
                        cmd.addFloat64(m_waypoint_lin_gain);
                        m_port_commands_output.write(cmd, ans);
                    }
                    sendWaypoint();
                }
            }
            else if (m_inner_controller.m_inner_status == navigation_status_preparing_before_move)
            {
                //do nothing, just wait
            }
            else if (m_inner_controller.m_inner_status == navigation_status_moving)
            {
                //do nothing, just wait
            }
            else if (m_inner_controller.m_inner_status == navigation_status_waiting_obstacle)
            {
                //do nothing, just wait
            }
            else if (m_inner_controller.m_inner_status == navigation_status_failing)
            {
                if (m_enable_try_recovery)
                {
                    if (m_recovery_attempt < m_max_recovery_attempts)
                    {
                        //try to avoid obstacles
                        m_recovery_attempt++;
                        yCWarning(PATHPLAN_CTRL, "unable to reach next waypoint, trying new solution.");
                        yCInfo   (PATHPLAN_CTRL, "This is recovery_attempt: %d/%d", m_recovery_attempt, m_max_recovery_attempts);

                        Bottle cmd, ans;
                        cmd.addString("stop");
                        m_port_commands_output.write(cmd, ans);

                        //update the map with the new obstacles
                        map_utilites::update_obstacles_map(m_current_map, m_temporary_obstacles_map);
                        //the following enlargement is done in order to take away the robot from the obstacles where it is stuck
                        m_temporary_obstacles_map.enlargeObstacles(0.1);
                        //search for a new path
                        if (!recomputePath())
                        {
                            yCInfo(PATHPLAN_CTRL, "Unable to recompute the path, aborting navigation");
                            abortNavigation();
                        }
                    }
                    else
                    {
                        yCInfo(PATHPLAN_CTRL, "Recovery_attempt: %d/%d, unable to find a solution, aborting navigation. ", m_recovery_attempt, m_max_recovery_attempts);
                        abortNavigation();
                    }
                }
                else
                {
                    //terminate navigation
                    yCError(PATHPLAN_CTRL, "unable to reach next waypoint, aborting navigation");
                    abortNavigation();
                }
            }
            else if (m_inner_controller.m_inner_status == navigation_status_aborted)
            {
                //terminate navigation
                m_planner_status = navigation_status_aborted;
                yCError(PATHPLAN_CTRL, "unable to reach next waypoint, aborting navigation");
                //current_path.clear();
            }
            else if (m_inner_controller.m_inner_status == navigation_status_error)
            {
                yCError(PATHPLAN_CTRL, "PathPlanner in error status");
                m_planner_status = navigation_status_error;
            }
            else if (m_inner_controller.m_inner_status == navigation_status_idle)
            {
                //send the first waypoint
                m_current_path_iterator = m_current_path->begin();
                yCInfo(PATHPLAN_CTRL, "sending the first waypoint");

                //send the tolerance to the inner controller
                {
                    Bottle cmd1, ans1;
                    cmd1.addString("set");
                    cmd1.addString("linear_tol");
                    cmd1.addFloat64(m_waypoint_tolerance_lin);
                    m_port_commands_output.write(cmd1, ans1);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("angular_tol");
                    cmd.addFloat64(m_waypoint_tolerance_ang);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("max_lin_speed");
                    cmd.addFloat64(m_waypoint_max_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("max_ang_speed");
                    cmd.addFloat64(m_waypoint_max_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("min_lin_speed");
                    cmd.addFloat64(m_waypoint_min_lin_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("min_ang_speed");
                    cmd.addFloat64(m_waypoint_min_ang_speed);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("ang_speed_gain");
                    cmd.addFloat64(m_waypoint_ang_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set");
                    cmd.addString("lin_speed_gain");
                    cmd.addFloat64(m_waypoint_lin_gain);
                    m_port_commands_output.write(cmd, ans);
                }
                sendWaypoint();
            }
            else
            {
                yCError(PATHPLAN_CTRL, "unrecognized inner status: %d", m_inner_controller.m_inner_status);
            }
        }
        break;
        case navigation_status_goal_reached:
        {
            //do nothing, just wait. After some time(1s), reset the status to navigation idle
            const double goal_reached_timeout = 1.0;
            if (yarp::os::Time::now() - m_navigation_started_at_timeX > goal_reached_timeout)
            {
                yCInfo(PATHPLAN_CTRL) << "Goal was reached "<< goal_reached_timeout<<" second(s) ago. On timeout expiration, navigation status was set idle";
                m_planner_status = navigation_status_idle;
            }
        }
        break;
        case navigation_status_waiting_obstacle:
        {
            //do nothing, just wait
        }
        break;
        case navigation_status_preparing_before_move:
        {
            //do nothing, just wait
        }
        break;
        case navigation_status_idle:
        {
            //do nothing, just wait
        }
        break;
        case  navigation_status_thinking:
        {
            //do nothing, just wait
        }
        break;
        case  navigation_status_paused:
        {
            //do nothing, just wait
        }
        break;
        case navigation_status_aborted:
        {
            //do nothing, just wait
        }
        break;
        case navigation_status_failing:
        {
            //do nothing, just wait.
            //this status should be not reached by the high-level planner.
        }
        break;
        case navigation_status_error:
        {
            if (m_inner_controller.m_inner_status != navigation_status_error)
            {
                m_planner_status = navigation_status_aborted;
            }
        }
        break;
        default:
        {
            //unknown status
            yCError(PATHPLAN_CTRL, "m_planner_status: unknown status:%d", m_planner_status);
            m_planner_status = navigation_status_error;
        }
        break;
    }

    //broadcast the planner status
    if (m_port_status_output.getOutputCount()>0)
    {
        string s = INavigation2DHelpers::statusToString(m_planner_status);
        Bottle &b = m_port_status_output.prepare();
        b.clear();
        b.addString(s.c_str());
        m_port_status_output.write();
    }
    
    m_mutex.post();
}

bool PlannerThread::getCurrentWaypoint(Map2DLocation &loc) const
{
    if (m_current_path == nullptr) return false;

    if (m_current_path_iterator != m_current_path->end())
    {
        loc = *m_current_path_iterator;
        return true;
    }

    return false;
}

bool PlannerThread::getCurrentWaypoint(XYCell &c) const
{
    if (m_current_path == nullptr) return false;

    if (m_current_path_iterator != m_current_path->end())
    {
        c = m_current_map.toXYCell(*m_current_path_iterator);
        return true;
    }
    return true;
}

bool PlannerThread::getCurrentMap(MapGrid2D& map) const
{
    map = m_current_map;
    return true;
}

bool PlannerThread::reloadCurrentMap()
{
    yCDebug(PATHPLAN_CTRL, "Reloading map %s from server", m_current_map.m_map_name.c_str());
    bool map_get_succesfull = this->m_iMap->get_map(m_localization_data.map_id, m_current_map);
    if (map_get_succesfull)
    {
        m_temporary_obstacles_map_mutex.lock();
        m_temporary_obstacles_map = m_current_map;
        m_temporary_obstacles_map_mutex.unlock();
        yCInfo(PATHPLAN_CTRL) << "Map '" << m_localization_data.map_id << "' successfully obtained from server";
        m_current_map.enlargeObstacles(m_robot_radius);
        m_augmented_map = m_current_map;
        yCDebug(PATHPLAN_CTRL, ) << "Obstacles enlargement performed (" << m_robot_radius << "m)";
        return true;
    }
    else
    {
        yCError(PATHPLAN_CTRL) << "Unable to get map '" << m_localization_data.map_id << "' from map server";
        std::vector<string> names_vector;
        m_iMap->get_map_names(names_vector);
        string names = "Known maps are:";
        for (auto it = names_vector.begin(); it != names_vector.end(); it++)
        {
            names = names + " " + (*it);
        }
        yCInfo(PATHPLAN_CTRL) << names;
        return false;
    }
    return true;
}

bool  PlannerThread::getCurrentPath(yarp::dev::Nav2D::Map2DPath& current_path) const
{
    if (m_current_path != nullptr)
    {
#if 0
        current_path = *m_current_path;
#elif 0
        current_path.clear();
        for (auto it = m_remaining_path.begin(); it != m_remaining_path.end(); it++)
        {
            current_path.push_back(*it);
        }
#else
        current_path.clear();
        for (auto it = m_current_path_iterator; it != m_current_path->end(); it++)
        {
            current_path.push_back(*it);
        }
#endif
        return true;
    }
    return false;
}

bool PlannerThread::getOstaclesMap(MapGrid2D& obstacles_map) 
{
    m_temporary_obstacles_map_mutex.lock();
    obstacles_map = m_temporary_obstacles_map;
    m_temporary_obstacles_map_mutex.unlock();
    return true;
}

void PlannerThread::sendWaypoint()
{
    size_t path_size = m_current_path->size();
    if (path_size==0)
    {
        yCWarning (PATHPLAN_CTRL, "Path queue is empty!");
        m_planner_status = navigation_status_idle;
        return;
    }
    //get the current waypoint from the list
    XYCell current_waypoint;
    if (getCurrentWaypoint(current_waypoint)==false)
    {
        yCError(PATHPLAN_CTRL, "getCurrentWaypoint failed!");
        m_planner_status = navigation_status_idle;
        return;
    }

    //send the waypoint to the inner controller
    Map2DLocation loc;
    loc.map_id = m_current_map.getMapName();
    loc.x = m_current_map.cell2World(current_waypoint).x;
    loc.y = m_current_map.cell2World(current_waypoint).y;
    loc.theta = std::nan("");
    if (path_size == 1 && std::isnan(m_final_goal.theta) == false)
    {
        //add the orientation to the last waypoint
        loc.theta = m_final_goal.theta;
    }
    yCDebug(PATHPLAN_CTRL, "sending command: %s", loc.toString().c_str());
    m_inner_controller.m_iInnerNav_target->gotoTargetByAbsoluteLocation(loc);

    //get inner navigation status
    NavigationStatusEnum inner_status;
    m_inner_controller.m_iInnerNav_ctrl->getNavigationStatus(inner_status);
    m_inner_controller.m_inner_status = inner_status;
}

void PlannerThread::sendFinalGoal()
{
    if (std::isnan(m_final_goal.theta) == false)
    {
        //add the orientation to the last waypoint
        yCDebug(PATHPLAN_CTRL, );
    }
    yCDebug(PATHPLAN_CTRL, "sending command: %s", m_final_goal.toString().c_str());
    m_inner_controller.m_iInnerNav_target->gotoTargetByAbsoluteLocation(m_final_goal);

    //get inner navigation status
    NavigationStatusEnum inner_status;
    m_inner_controller.m_iInnerNav_ctrl->getNavigationStatus(inner_status);
    m_inner_controller.m_inner_status = inner_status;
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
        yCError(PATHPLAN_CTRL) << "PlannerThread::startPath() current robot location (" << start_vec.toString() << ")is not inside map" << m_current_map.getMapName();
        return false;
    }
    if (m_current_map.isInsideMap(goal_vec) == false)
    {
        yCError(PATHPLAN_CTRL) << "PlannerThread::startPath() requested goal (" << goal_vec.toString() << ") is not inside map" << m_current_map.getMapName();
        return false;
    }
    XYCell goal = m_current_map.world2Cell(goal_vec);
    XYCell start = m_current_map.world2Cell(start_vec);
#ifdef DEBUG_WITH_CELLS
    start.x = 150;//&&&&&
    start.y = 150;//&&&&&
#endif
    double t1 = yarp::os::Time::now();
    //clear the memory 
    m_computed_path.clear();
    m_computed_simplified_path.clear();
    m_planner_status = navigation_status_thinking;

    //search for a path
    bool b = map_utilites::findPath(m_current_map, start, goal, m_computed_path);
    if (!b)
    {
        yCError (PATHPLAN_CTRL, "path not found");
        m_planner_status = navigation_status_aborted;
        return false;
    }
    double t2 = yarp::os::Time::now();

    //search for an simpler path (waypoint optimization)
    map_utilites::simplifyPath(m_current_map, m_computed_path, m_computed_simplified_path);
    yCInfo(PATHPLAN_CTRL, "path size:%d simplified path size:%d time: %.2f", (int)m_computed_path.size(), (int)m_computed_simplified_path.size(), t2 - t1);

    //choose the path to use
    if (m_use_optimized_path)
    {
        m_current_path = &m_computed_simplified_path;
    }
    else
    {
        m_current_path = &m_computed_path;
    }

    //check if the size of the path. This is needed to allow in-place rotations only.
    if (m_current_path->size() == 0)
    {
        double threshold = 1.0; //deg
        if (fabs(m_localization_data.theta - m_sequence_of_goals.front().theta) > threshold)
        {
           yCWarning(PATHPLAN_CTRL) << "Requested path has zero length. Adding waypoint with final orientation";
           m_current_path->push_back(m_sequence_of_goals.front());
        }
        else
        {
            yCWarning(PATHPLAN_CTRL) << "Requested path has zero length. Aborting;";
            m_planner_status = navigation_status_goal_reached;
            return true;
        }
    }

    m_current_path_iterator = m_current_path->begin();
    std::copy(m_current_path->begin(), m_current_path->end(), std::back_inserter(m_remaining_path));

    //debug print
    if (1)
    {
        yCDebug(PATHPLAN_CTRL) << "Current pos" << " x:" << start_vec.x << " y:" << start_vec.y;
        yCDebug(PATHPLAN_CTRL) << m_current_path->toString();
        yCDebug(PATHPLAN_CTRL) << "Final goal" << " x:" << goal_vec.x << " y:" << goal_vec.y << " t:" << m_sequence_of_goals.front().theta;
    }

    //just set the status to moving, do not set position commands.
    //The waypoint is set in the main 'run' loop.
    m_planner_status = navigation_status_moving;
    m_navigation_started_at_timeX = yarp::os::Time::now();
    return true;
}

bool PlannerThread::recomputePath()
{
    if (getNavigationStatusAsInt() == navigation_status_idle)
    {
        yCError(PATHPLAN_CTRL) << "Unable to recompute path. Navigation task not assigned yet.";
        return false;
    }
    if (getNavigationStatusAsInt() == navigation_status_paused)
    {
        yCError(PATHPLAN_CTRL) << "Unable to recompute path. Navigation task is currently paused.";
        return false;
    }
    if (getNavigationStatusAsInt() == navigation_status_goal_reached)
    {
        yCError(PATHPLAN_CTRL) << "Unable to recompute path. Navigation Goal has been already reached.";
        return false;
    }
    if (getNavigationStatusAsInt() == navigation_status_thinking)
    {
        yCError(PATHPLAN_CTRL) << "Unable to recompute path. A navigation plan is already under computation.";
        return false;
    }

    Map2DLocation loc;
    bool b = true;
    b &= getCurrentAbsTarget(loc);
    //@@@ check timing here
    yarp::os::Time::delay(0.2);
    b &= stopMovement();
    //@@@ check timing here
    yarp::os::Time::delay(0.2);
    b &= setNewAbsTarget(loc);

    return b;
}

void PlannerThread::abortNavigation()
{
    Bottle cmd, ans;
    cmd.addString("stop");
    m_port_commands_output.write(cmd, ans);
    m_planner_status = navigation_status_aborted;
}