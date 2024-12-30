/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pathPlannerCtrl.h"
#include "pathPlannerCtrlHelpers.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(PATHPLAN_GETS, "navigation.devices.robotPathPlanner.gets")

bool PlannerThread::getFinalAbsTarget(Map2DLocation& target)
{
    Map2DLocation m;
    //TODO: check for target validity
    m.map_id = m_sequence_of_goals.back().map_id;
    m.x = m_sequence_of_goals.back().x;
    m.y = m_sequence_of_goals.back().y;
    m.theta = m_sequence_of_goals.back().theta;
    target = m;
    return true;
}

bool PlannerThread::getFinalRelTarget(Map2DLocation& target)
{
    Map2DLocation m;
    //TODO: check for target validity
    m.map_id = m_sequence_of_goals.back().map_id;
    m.x = m_sequence_of_goals.back().x - m_localization_data.x;
    m.y = m_sequence_of_goals.back().y - m_localization_data.y;
    m.theta = m_sequence_of_goals.back().theta - m_localization_data.theta;
    target = m;
    return true;
}

bool PlannerThread::getCurrentAbsTarget(Map2DLocation& target)
{
    Map2DLocation m;
    if (m_sequence_of_goals.size()==0)
    {
        target.map_id="invalid";
        yCError(PATHPLAN_GETS) << "No valid target has been set yet.";
        return false;
    }
    m.map_id = m_sequence_of_goals.front().map_id;
    m.x = m_sequence_of_goals.front().x;
    m.y = m_sequence_of_goals.front().y;
    m.theta = m_sequence_of_goals.front().theta;
    target = m ;
    return true;
}

bool PlannerThread::getCurrentRelTarget(Map2DLocation& target)
{
    Map2DLocation m;
    if (m_sequence_of_goals.size()==0)
    {
        target.map_id="invalid";
        yCError(PATHPLAN_GETS) << "No valid target has been set yet.";
        return false;
    }
    m.map_id = m_sequence_of_goals.front().map_id;
    m.x = m_sequence_of_goals.front().x - m_localization_data.x;
    m.y = m_sequence_of_goals.front().y - m_localization_data.y;
    m.theta = m_sequence_of_goals.front().theta - m_localization_data.theta;
    target = m;
    return true;
}

bool PlannerThread::getCurrentPos(Map2DLocation& v)
{
    v.map_id = m_localization_data.map_id;
    v.x = m_localization_data.x;
    v.y = m_localization_data.y;
    v.theta = m_localization_data.y;
    return true;
}

string PlannerThread::getFinalMapId()
{
    return  m_sequence_of_goals.back().map_id;
}

string PlannerThread::getCurrentMapId()
{
    return m_current_map.getMapName();
}

void  PlannerThread::getTimeouts(int& localiz, int& laser, int& inner_status)
{
    localiz = m_loc_timeout_counter;
    laser = m_laser_timeout_counter;
    inner_status = m_inner_controller.m_inner_status_timeout_counter;
}
