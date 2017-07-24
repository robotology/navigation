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
#include "pathPlannerHelpers.h"

using namespace std;
using namespace yarp::dev;

bool PlannerThread::setNewAbsTarget(yarp::dev::Map2DLocation target)
{
    if (m_planner_status != navigation_status_idle &&
        m_planner_status != navigation_status_goal_reached &&
        m_planner_status != navigation_status_aborted &&
        m_planner_status != navigation_status_failing)
    {
        yError ("Not in idle state, send a 'stop' first\n");
        return false;
    }

    m_final_goal = target;

    if (target.map_id == m_current_map.getMapName())
    {
        std::queue<yarp::dev::Map2DLocation> empty;
        std::swap(m_sequence_of_goals, empty);
        m_sequence_of_goals.push(m_final_goal);
        if (startPath())
        {
            return true;
        }
        else
        {
            yError() << "PlannerThread::setNewAbsTarget() Unable to start path";
            return false;
        }
    }
    else
    {
        yError("Requested goal is not in the current map!\n");
        //here I need to:
        //1) find in the location server a connection between target.map_id and m_current_map.m_map_name
        //2) obtain a list of targets, put them in a queue.
        //3) startNewPath with this first element of the queue
        std::queue<yarp::dev::Map2DLocation> empty;
        std::swap(m_sequence_of_goals, empty);
        m_sequence_of_goals.push(m_final_goal);
        return false;
    }
    return true;
}

bool PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
    if(target.size() != 3)
    {
        yError() << "PlannerThread::setNewRelTarget() invalid target vector size";
        return false;
    }
    //target and localization data are formatted as follows: x, y, angle (in degrees)
    if (m_planner_status != navigation_status_idle &&
        m_planner_status != navigation_status_goal_reached &&
        m_planner_status != navigation_status_aborted &&
        m_planner_status != navigation_status_failing)
    {
        yError ("Not in idle state, send a 'stop' first");
        return false;
    }
    yDebug() << "received new relative target at:" << target[0] << target[1] << target[2];

    double a = m_localization_data.theta * DEG2RAD;
    yDebug() << "current position:" << m_localization_data.x << m_localization_data.y << m_localization_data.theta;
    //this is the inverse of the tranformation matrix from world to robot
    m_final_goal.x = +target[0] * cos(a) - target[1] * sin(a) + m_localization_data.x;
    m_final_goal.y = +target[0] * sin(a) + target[1] * cos(a) + m_localization_data.y;
    m_final_goal.theta = target[2] + m_localization_data.theta;
    m_final_goal.map_id = this->getCurrentMapId();
    std::queue<yarp::dev::Map2DLocation> empty;
    std::swap(m_sequence_of_goals, empty);
    m_sequence_of_goals.push(m_final_goal);
    if (startPath())
    {
        return true;
    }
    else
    {
        yError() << "PlannerThread::setNewRelTarget() Unable to start path";
        return false;
    }
}

void PlannerThread::stopMovement()
{
    //stop the inner navigation loop
    Bottle cmd1, ans1;
    cmd1.addString("stop"); 
    m_port_commands_output.write(cmd1, ans1);

    //stop the outer navigation loop


    if (m_planner_status != navigation_status_idle)
    {
        m_planner_status = navigation_status_idle;
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
    m_port_commands_output.write(cmd1, ans1);

    //stop the outer navigation loop


    if (m_planner_status != navigation_status_moving)
    {
        m_planner_status = navigation_status_moving;
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
    m_port_commands_output.write(cmd1, ans1);

    //stop the outer navigation loop

    if (m_planner_status != navigation_status_paused)
    {
        m_planner_status = navigation_status_paused;
        yInfo ("Navigation stopped");
    }
    else
    {
        yWarning ("Already paused");
    }
}

void PlannerThread::gotoLocation(std::string location_name)
{
    yarp::dev::Map2DLocation loc;
    if(m_iMap->getLocation(location_name,loc))
    {
        setNewAbsTarget(loc);
    }
    updateLocations();
}

void PlannerThread::storeCurrentLocation(std::string location_name)
{
    m_iMap->storeLocation(location_name,m_localization_data);
    updateLocations();
}

void PlannerThread::deleteLocation(std::string location_name)
{
    m_iMap->deleteLocation(location_name);
    updateLocations();
}

void PlannerThread::printStats()
{
}

int PlannerThread::getNavigationStatusAsInt()
{
    return m_planner_status;
}

string PlannerThread::getNavigationStatusAsString()
{
    string s = pathPlannerHelpers::getStatusAsString(m_planner_status);
    return s;
}

