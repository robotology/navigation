/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pathPlannerCtrl.h"
#include "pathPlannerCtrlHelpers.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(PATHPLAN_ACTIONS, "navigation.devices.robotPathPlanner.actions")

void PlannerThread::resetAttemptCounter()
{
    m_recovery_attempt=0;
}

bool PlannerThread::setNewAbsTarget(Map2DLocation target)
{
    if (m_planner_status != navigation_status_idle &&
        m_planner_status != navigation_status_goal_reached &&
        m_planner_status != navigation_status_aborted &&
        m_planner_status != navigation_status_failing)
    {
        yCError (PATHPLAN_ACTIONS,"Not in idle state, send a 'stop' first\n");
        return false;
    }

    yCInfo(PATHPLAN_ACTIONS) << "Received a new target:" << target.toString() << ", attempt:" << m_recovery_attempt;
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
            yCError(PATHPLAN_ACTIONS) << "PlannerThread::setNewAbsTarget() Unable to start path";
            return false;
        }
    }
    else
    {
        yCWarning(PATHPLAN_ACTIONS, "Requested goal is not in the current map!\n");
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
        yCError(PATHPLAN_ACTIONS) << "PlannerThread::setNewRelTarget() invalid target vector size";
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
        yCError (PATHPLAN_ACTIONS, "Not in idle state, send a 'stop' first");
        return false;
    }

    yCInfo(PATHPLAN_ACTIONS) << "received new relative target at:" << target[0] << target[1] << target[2] << ", attempt:" << m_recovery_attempt;

    double a = m_localization_data.theta * DEG2RAD;
    yCDebug(PATHPLAN_ACTIONS) << "current position:" << m_localization_data.x << m_localization_data.y << m_localization_data.theta;
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
        yCError(PATHPLAN_ACTIONS) << "PlannerThread::setNewRelTarget() Unable to start path";
        return false;
    }
}

bool PlannerThread::stopMovement()
{
    bool ret = true;
    //stop the inner navigation loop
    m_inner_controller.m_iInnerNav_ctrl->stopNavigation();

    //stop the outer navigation loop
    if (m_planner_status != navigation_status_idle)
    {
        m_planner_status = navigation_status_idle;
        yCInfo (PATHPLAN_ACTIONS, "Navigation stopped");
    }
    else
    {
        yCWarning (PATHPLAN_ACTIONS, "Already not moving");
        ret = false;
    }
    return ret;
}

bool PlannerThread::resumeMovement()
{
    bool ret = true;
    //resuming the inner navigation loop
    m_inner_controller.m_iInnerNav_ctrl->resumeNavigation();

    //resume the outer navigation loop
    if (m_planner_status != navigation_status_moving)
    {
        m_planner_status = navigation_status_moving;
        yCInfo (PATHPLAN_ACTIONS, "Navigation resumed");
    }
    else
    {
        yCWarning (PATHPLAN_ACTIONS, "Already moving!");
        ret = false;
    }
   return ret;
}

bool PlannerThread::pauseMovement(double d)
{
    bool ret = true;
    //pausing the inner navigation loop
    m_inner_controller.m_iInnerNav_ctrl->suspendNavigation(d);

    //pausing the outer navigation loop
    if (m_planner_status != navigation_status_paused)
    {
        m_planner_status = navigation_status_paused;
        yCInfo (PATHPLAN_ACTIONS, "Navigation stopped");
    }
    else
    {
        yCWarning (PATHPLAN_ACTIONS, "Already paused");
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
    string s = INavigation2DHelpers::statusToString(m_planner_status);
    return s;
}

