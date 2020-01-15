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
#include "pathPlannerCtrlHelpers.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

bool PlannerThread::setNewAbsTarget(Map2DLocation target)
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
        //this a trick to clean the queue
        std::queue<Map2DLocation> empty;
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
        std::queue<Map2DLocation> empty;
        std::swap(m_sequence_of_goals, empty);
        m_sequence_of_goals.push(m_final_goal);
        return false;
    }
    return true;
}

bool PlannerThread::setNewRelTarget(yarp::sig::Vector target)
{
    if(target.size() != 2 && target.size() != 3)
    {
        yError() << "PlannerThread::setNewRelTarget() invalid target vector size";
        return false;
    }

    if (target.size() == 2)
    {
        target.push_back(std::nan(""));
        //This is done intentionally, in order to have m_final_goal.theta = nan, too.
        //m_final_goal.theta = nan is properly handled by function PlannerThread::sendWaypoint()
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
    //this is the inverse of the transformation matrix from world to robot
    m_final_goal.x = +target[0] * cos(a) - target[1] * sin(a) + m_localization_data.x;
    m_final_goal.y = +target[0] * sin(a) + target[1] * cos(a) + m_localization_data.y;
    m_final_goal.theta = target[2] + m_localization_data.theta;
    m_final_goal.map_id = this->getCurrentMapId();
    std::queue<Map2DLocation> empty;
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

bool PlannerThread::stopMovement()
{
    bool ret = true;
    //stop the inner navigation loop
    m_iInnerNav_ctrl->stopNavigation();

    //stop the outer navigation loop
    if (m_planner_status != navigation_status_idle)
    {
        m_planner_status = navigation_status_idle;
        yInfo ("Navigation stopped");
    }
    else
    {
        yWarning ("Already not moving");
        ret = false;
    }
    return ret;
}

bool PlannerThread::resumeMovement()
{
    bool ret = true;
    //resuming the inner navigation loop
    m_iInnerNav_ctrl->resumeNavigation();

    //resume the outer navigation loop
    if (m_planner_status != navigation_status_moving)
    {
        m_planner_status = navigation_status_moving;
        yInfo ("Navigation resumed");
    }
    else
    {
        yWarning ("Already moving!");
        ret = false;
    }
   return ret;
}

bool PlannerThread::pauseMovement(double d)
{
    bool ret = true;
    //pausing the inner navigation loop
    m_iInnerNav_ctrl->suspendNavigation(d);

    //pausing the outer navigation loop
    if (m_planner_status != navigation_status_paused)
    {
        m_planner_status = navigation_status_paused;
        yInfo ("Navigation stopped");
    }
    else
    {
        yWarning ("Already paused");
        ret = false;
    }
    return ret;
}

void PlannerThread::printStats()
{
}

NavigationStatusEnum PlannerThread::getNavigationStatusAsInt()
{
    return m_planner_status;
}

string PlannerThread::getNavigationStatusAsString()
{
    string s = yarp::dev::INavigation2DHelpers::statusToString(m_planner_status);
    return s;
}

