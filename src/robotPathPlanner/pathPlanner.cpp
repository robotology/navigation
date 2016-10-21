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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

NavigationStatusEnum string2status(string s)
{
    //enum status_type {IDLE=0, MOVING, WAITING_OBSTACLE, REACHED, ABORTED, PAUSED};
    NavigationStatusEnum status;
    if (s == "navigation_status_idle")     status = navigation_status_idle;
    else if (s == "navigation_status_moving")   status = navigation_status_moving;
    else if (s == "navigation_status_waiting_obstacle")  status = navigation_status_waiting_obstacle;
    else if (s == "navigation_status_goal_reached")  status = navigation_status_goal_reached;
    else if (s == "navigation_status_aborted")  status = navigation_status_aborted;
    else if (s == "navigation_status_paused")   status = navigation_status_paused;
    else 
    {
        yError("PlannerThread::string2status(): Unknown status of inner controller: '%s'!", s.c_str());
        status = navigation_status_idle;
    }
    return status;
}

std::string getStatusAsString(NavigationStatusEnum status)
{
    if (status == navigation_status_idle) return std::string("navigation_status_idle");
    else if (status == navigation_status_moving) return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle) return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached) return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted) return std::string("navigation_status_aborted");
    else if (status == navigation_status_paused) return std::string("navigation_status_paused");

    yError("PlannerThread::getStatusAsString(): Unknown status of inner controller: '%d'!", status);
    return std::string("unknown");
}

void PlannerThread::select_optimized_path(bool b)
{
    if (b)
    {
        yInfo ("using optimized path planning");
        use_optimized_path = true;
        current_path=&computed_simplified_path;
    }
    else
    {
        yInfo("using raw path planning");
        use_optimized_path = false;
        current_path=&computed_path;
    }
}

Map2DLocation PlannerThread::getCurrentAbsTarget()
{
    return Map2DLocation(frame_map_id, goal_data[0], goal_data[1], goal_data[2]);
}

Map2DLocation PlannerThread::getCurrentRelTarget()
{
    return Map2DLocation(frame_robot_id, goal_data[0]-localization_data[0], goal_data[1]-localization_data[1], goal_data[2]-localization_data[2]);
}

void PlannerThread::getCurrentPos(yarp::sig::Vector& v)
{
    v.resize(localization_data.size());
    v = localization_data;
}

string PlannerThread::getMapId()
{
    return frame_map_id;
}

void PlannerThread::run()
{
    mutex.wait();

    //read a target set from a yarpview
    yarp::os::Bottle *gui_targ = port_yarpview_target_input.read(false);
    if (gui_targ)
    {
        cell c;
        c.x=(*gui_targ).get(0).asInt();
        c.y=(*gui_targ).get(1).asInt();
        yarp::sig::Vector v = map.cell2world(c);
        yInfo ("selected point is located at (%6.3f, %6.3f)", v[0], v[1]);
        yarp::os::Bottle& out = port_yarpview_target_output.prepare();
        out.clear();
        out.addString("gotoAbs");
        out.addDouble(v[0]);
        out.addDouble(v[1]);
        port_yarpview_target_output.write();
    }

    //read the localization data
    if (use_localization_from_port)
    {
        yarp::sig::Vector *loc = port_localization_input.read(false);
        if (loc)
        {
            localization_data = *loc;
            loc_timeout_counter = 0;
        }
        else
        {
            loc_timeout_counter++;
            if (loc_timeout_counter>TIMEOUT_MAX) loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else if (use_localization_from_tf)
    {
        yarp::sig::Vector iv;
        yarp::sig::Vector pose;
        iv.resize(6, 0.0);
        pose.resize(6, 0.0);
        bool r = iTf->transformPose(frame_robot_id, frame_map_id, iv, pose);
        if (r)
        {
            //data is formatted as follows: x, y, angle (in degrees)
            localization_data[0]     = pose[0];
            localization_data[1]     = pose[1];
            localization_data[2]     = pose[5] * RAD2DEG;
            loc_timeout_counter = 0;
        }
        else
        {
            loc_timeout_counter++;
            if (loc_timeout_counter > TIMEOUT_MAX) loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else
    {
        yWarning() << "Localization disabled";
    }

    //read the laser data
    yarp::sig::Vector scan;
    bool ret = iLaser->getMeasurementData(scan);

    if (ret)
    {
        laser_map_cell.clear();
        unsigned int scansize = scan.size();
        if (laser_data == 0)
        {
            laser_data = new lasermap_type[scansize];
        }
        for (unsigned int i = 0; i<scansize; i++)
        {
            double angle = (i / double(scansize)*laser_angle_of_view + robot_laser_t)* DEG2RAD;
            laser_data[i].x = scan[i] * cos(angle) + robot_laser_x;
            laser_data[i].y = scan[i] * sin(angle) + robot_laser_y;

            yarp::sig::Vector v(2);
            double ss = cos (localization_data[2] * DEG2RAD);
            double cs = sin (localization_data[2] * DEG2RAD);
            v[0] = laser_data[i].x*cs - laser_data[i].y*ss + localization_data[0];
            v[1] = laser_data[i].x*ss + laser_data[i].y*cs + localization_data[1];
            cell tmp_cell = map.world2cell(v);
            laser_map_cell.push_back(tmp_cell);
        }
        laser_timeout_counter=0;
    }
    else
    {
        laser_timeout_counter++;
    }

    //read the internal navigation status
    Bottle cmd1, ans1;
    cmd1.addString("get"); 
    cmd1.addString("navigation_status");
    port_commands_output.write(cmd1,ans1);
    string s = ans1.get(0).toString().c_str();
    inner_status = string2status(s);
    /*yarp::os::Bottle *st = port_status_input.read(true);
    if (st)
    {
        string s = st->get(0).toString().c_str();
        inner_status_timeout_counter=0;
        //convet s to inner status
        inner_status = string2status(s);
    }
    else inner_status_timeout_counter++;
    */

    //check if the next waypoint has to be sent
    int path_size = current_path->size();
    if (planner_status == navigation_status_moving)
    {
        if (inner_status == navigation_status_goal_reached)
        {
            yInfo ("waypoint reached");
            if (path_size == 0)
            {
                //navigation is complete
                yInfo("navigation complete");
                planner_status = navigation_status_goal_reached;
            }
            else if (path_size == 1)
            {
                 //send the next waypoint
                yInfo("sending the last waypoint");

                //send the tolerance to the inner controller
                Bottle cmd1, ans1;
                cmd1.addString("set"); 
                cmd1.addString("linear_tol");
                cmd1.addDouble(goal_tolerance_lin);
                port_commands_output.write(cmd1,ans1);

                Bottle cmd2, ans2;
                cmd2.addString("set"); 
                cmd2.addString("angular_tol");
                cmd2.addDouble(goal_tolerance_ang);
                port_commands_output.write(cmd2,ans2);

                //last waypoint ha minimum linear speed = 0
                Bottle cmd3, ans3;
                cmd3.addString("set"); 
                cmd3.addString("min_lin_speed");
                cmd3.addDouble(0.0);
                port_commands_output.write(cmd3,ans3);

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
                    cmd.addDouble(max_lin_speed);
                    port_commands_output.write(cmd,ans);
                }
                {
                    Bottle cmd, ans;
                    cmd.addString("set"); 
                    cmd.addString("max_lin_speed");
                    cmd.addDouble(max_lin_speed);
                    port_commands_output.write(cmd,ans);
                }
            }
        }
        else if (inner_status == navigation_status_moving)
        {
            //do nothing, just wait
        }
        else if (inner_status == navigation_status_waiting_obstacle)
        {
            //do nothing, just wait
        }
        else if (inner_status == navigation_status_aborted)
        {
            //terminate navigation
            planner_status = navigation_status_aborted;
            yError ("unable to reach next waypoint, aborting navigation");
            //current_path.clear();
        }
        else if (inner_status == navigation_status_idle)
        {
            //send the first waypoint
            yInfo ("sending the first waypoint");

            //send the tolerance to the inner controller
            {
                Bottle cmd1, ans1;
                cmd1.addString("set"); 
                cmd1.addString("linear_tol");
                cmd1.addDouble(waypoint_tolerance_lin);
                port_commands_output.write(cmd1,ans1);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("angular_tol");
                cmd.addDouble(waypoint_tolerance_ang);
                port_commands_output.write(cmd,ans);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("max_lin_speed");
                cmd.addDouble(max_lin_speed);
                port_commands_output.write(cmd,ans);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("max_ang_speed");
                cmd.addDouble(max_ang_speed);
                port_commands_output.write(cmd,ans);
            }

            {
                //here I send min_lin_speed = max_lin_speed to have constant velocity
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_lin_speed");
                cmd.addDouble(max_lin_speed);
                port_commands_output.write(cmd,ans);
            }

            {
                Bottle cmd, ans;
                cmd.addString("set"); 
                cmd.addString("min_ang_speed");
                cmd.addDouble(min_ang_speed);
                port_commands_output.write(cmd,ans);
                sendWaypoint();
            }
        }
        else
        {
            yError ("unrecognized inner status: %d", inner_status);
        }
    }
    else if (planner_status == navigation_status_goal_reached)
    {
        //do nothing, just wait
    }
    else if (planner_status == navigation_status_idle)
    {
        //do nothing, just wait
    }
    else if (planner_status == navigation_status_thinking)
    {
        //do nothing, just wait
    }
    else if (planner_status == navigation_status_aborted)
    {
        //do nothing, just wait
    }
    else
    {
        //unknown status
        yError ("unknown status:%d", planner_status);
    }

    //broadcast the planner status
    if (port_status_output.getOutputCount()>0)
    {
        string s = getStatusAsString(planner_status);
        Bottle &b=this->port_status_output.prepare();
        b.clear();
        b.addString(s.c_str());
        port_status_output.write();
    }
    
    //draw the map
    static CvScalar blue_color  = cvScalar(0,0,200);
    static CvScalar blue_color2 = cvScalar(80,80,200);
    cell start = map.world2cell(localization_data);

    cvCopyImage(map.processed_map, map.processed_map_with_scan);
    if (laser_timeout_counter<TIMEOUT_MAX)
    {
        map.drawLaserScan(map.processed_map_with_scan,laser_map_cell,blue_color);
        //map.enlargeScan(laser_map_cell,6);
        //map.drawLaserScan(map.processed_map_with_scan,laser_map_cell,blue_color2);
    }

    map.drawCurrentPosition(map.processed_map_with_scan,start,blue_color);
    static IplImage* map_with_path = 0;
    if (map_with_path==0) map_with_path = cvCloneImage(map.processed_map_with_scan);
    else cvCopyImage(map.processed_map_with_scan,map_with_path);

    CvScalar color = cvScalar(0,200,0);
    CvScalar color2 = cvScalar(0,200,100);

    if (planner_status != navigation_status_idle && planner_status != navigation_status_goal_reached)
    {
#ifdef DRAW_BOTH_PATHS
        map.drawPath(map_with_path, start, computed_path, color); 
        map.drawPath(map_with_path, start, computed_simplified_path, color2);
#else
        map.drawPath(map_with_path, start, current_waypoint, *current_path, color); 
#endif
    }

    static IplImage* map_with_location = 0;
    if (map_with_location == 0) map_with_location = cvCloneImage(map_with_path);
    else cvCopyImage(map_with_path, map_with_location);



    map.sendToPort(&port_map_output,map_with_location);
    mutex.post();
}

void PlannerThread::sendWaypoint()
{
    int path_size = current_path->size();
    if (path_size==0)
    {
        yWarning ("Path queue is empty!");
        planner_status = navigation_status_idle;
        return;
    }
    //get the next waypoint from the list
    current_waypoint = current_path->front();
    current_path->pop();
    //send the waypoint to the inner controller
    Bottle cmd1, ans1;
    cmd1.addString("gotoAbs"); 
    yarp::sig::Vector v = map.cell2world(current_waypoint);
    cmd1.addDouble(v[0]);
    cmd1.addDouble(v[1]);
    if (path_size==1 && goal_data.size()==3)
    {
        //add the orientation to the last waypoint
        cmd1.addDouble(goal_data[2]);
    }
    yDebug ("sending command: %s", cmd1.toString().c_str());
    port_commands_output.write(cmd1,ans1);
    //yDebug ("received answer: %s", ans1.toString().c_str());

    Bottle cmd2, ans2;
    cmd2.addString("get");
    cmd2.addString("navigation_status");
    //yDebug ("sending command: %s", cmd2.toString().c_str());
    port_commands_output.write(cmd2,ans2);
    //yDebug ("received answer: %s", ans2.toString().c_str());
    inner_status = string2status(ans2.toString().c_str());
}

void PlannerThread::startNewPath(cell target)
{
    cell start = map.world2cell(localization_data);
#ifdef DEBUG_WITH_CELLS
    start.x = 150;//&&&&&
    start.y = 150;//&&&&&
#endif
    double t1 = yarp::os::Time::now();
    //clear the memory 
    std::queue<cell> empty;
    std::swap(computed_path, empty );
    std::queue<cell> empty2;
    std::swap( computed_simplified_path, empty2 );
    planner_status = navigation_status_thinking;

    //search for a path
    bool b = map.findPath(map.processed_map, start , target, computed_path);
    if (!b)
    {
        yError ("path not found");
        planner_status = navigation_status_aborted;
        return;
    }
    double t2 = yarp::os::Time::now();

    //search for an simpler path (waypoint optimization)
    map.simplifyPath(map.processed_map, computed_path, computed_simplified_path);
    yInfo ("path size:%d simplified path size:%d time: %.2f", computed_path.size(), computed_simplified_path.size(), t2-t1);

    //choose the path to use
    if (use_optimized_path)
    {
        current_path=&computed_simplified_path;
    }
    else
    {
        current_path=&computed_path;
    }

    //just set the status to moving, do not set position commands.
    //The wayypoint ist set in the main 'run' loop.
    planner_status = navigation_status_moving;
}

void PlannerThread::setNewAbsTarget(yarp::sig::Vector target)
{
    if (planner_status != navigation_status_idle &&
        planner_status != navigation_status_goal_reached &&
        planner_status != navigation_status_aborted)
    {
        yError ("Not in idle state, send a 'stop' first\n");
        return;
    }

    goal_data = target;
    cell goal = map.world2cell(target);
#ifdef DEBUG_WITH_CELLS
    goal.x = (int)target_data[0]; //&&&&&
    goal.y = (int)target_data[1]; //&&&&&
#endif
    startNewPath(goal);
}

void PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
    if(target.size() != 3)
    {
        yError() << "PlannerThread::setNewRelTarget invalid target vector size";
        return;
    }
    //target and localization data are formatted as follows: x, y, angle (in degrees)
    if (planner_status != navigation_status_idle &&
        planner_status != navigation_status_goal_reached &&
        planner_status != navigation_status_aborted)
    {
        yError ("Not in idle state, send a 'stop' first");
        return;
    }
    yDebug() << "received new relative target at:" << target[0] << target[1] << target[2];

    double a = localization_data[2] * DEG2RAD;
    yDebug() << "current position:" << localization_data[0] << localization_data[1] << localization_data[2];
    //this is the inverse of the tranformation matrix from world to robot
    goal_data[0] = +target[0] * cos(a) - target[1] * sin(a) + localization_data[0];
    goal_data[1] = +target[0] * sin(a) + target[1] * cos(a) + localization_data[1];
    goal_data[2] = target[2] + localization_data[2];
    cell goal = map.world2cell(goal_data);
    startNewPath(goal);
}

void PlannerThread::stopMovement()
{
    //stop the inner navigation loop
    Bottle cmd1, ans1;
    cmd1.addString("stop"); 
    port_commands_output.write(cmd1,ans1);

    //stop the outer navigation loop


    if (planner_status != navigation_status_idle)
    {
        planner_status = navigation_status_idle;
        yInfo ("Navigation stopped");
    }
    else
    {
        yWarning ("Already not moving");
    }
}

void PlannerThread::resumeMovement()
{
    //resuming the inner navigation loop
    Bottle cmd1, ans1;
    cmd1.addString("resume");
    port_commands_output.write(cmd1,ans1);

    //stop the outer navigation loop


    if (planner_status != navigation_status_moving)
    {
        planner_status = navigation_status_moving;
        yInfo ("Navigation resumed");
    }
    else
    {
        yWarning ("Already moving!");
    }
}

void PlannerThread::pauseMovement(double d)
{
    //pausing the inner navigation loop
    Bottle cmd1, ans1;
    cmd1.addString("pause");
    port_commands_output.write(cmd1,ans1);

    //stop the outer navigation loop

    if (planner_status != navigation_status_paused)
    {
        planner_status = navigation_status_paused;
        yInfo ("Navigation stopped");
    }
    else
    {
        yWarning ("Already paused");
    }
}

void PlannerThread::printStats()
{
}

int PlannerThread::getNavigationStatusAsInt()
{
    return planner_status;
}

string PlannerThread::getNavigationStatusAsString()
{
    string s = getStatusAsString(planner_status);
    return s;
}

