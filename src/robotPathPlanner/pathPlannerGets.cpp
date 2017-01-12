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

Map2DLocation PlannerThread::getFinalAbsTarget()
{
    Map2DLocation m;
    m.map_id = m_sequence_of_goals.back().map_id;
    m.x = m_sequence_of_goals.back().x;
    m.y = m_sequence_of_goals.back().y;
    m.theta = m_sequence_of_goals.back().theta;
    return m;
}

Map2DLocation PlannerThread::getFinalRelTarget()
{
    Map2DLocation m;
    m.map_id = m_sequence_of_goals.back().map_id;
    m.x = m_sequence_of_goals.back().x - m_localization_data.x;
    m.y = m_sequence_of_goals.back().y - m_localization_data.y;
    m.theta = m_sequence_of_goals.back().theta - m_localization_data.theta;
    return m;
}

Map2DLocation PlannerThread::getCurrentAbsTarget()
{
    Map2DLocation m;
    m.map_id = m_sequence_of_goals.front().map_id;
    m.x = m_sequence_of_goals.front().x;
    m.y = m_sequence_of_goals.front().y;
    m.theta = m_sequence_of_goals.front().theta;
    return m;
}

Map2DLocation PlannerThread::getCurrentRelTarget()
{
    Map2DLocation m;
    m.map_id = m_sequence_of_goals.front().map_id;
    m.x = m_sequence_of_goals.front().x - m_localization_data.x;
    m.y = m_sequence_of_goals.front().y - m_localization_data.y;
    m.theta = m_sequence_of_goals.front().theta - m_localization_data.theta;
    return m;
}

void PlannerThread::getCurrentPos(Map2DLocation& v)
{
    v.map_id = m_localization_data.map_id;
    v.x = m_localization_data.x;
    v.y = m_localization_data.y;
    v.theta = m_localization_data.y;
}

string PlannerThread::getFinalMapId()
{
    return  m_sequence_of_goals.back().map_id;
}

string PlannerThread::getCurrentMapId()
{
    return m_current_map.m_map_name;
}

void  PlannerThread::getTimeouts(int& localiz, int& laser, int& inner_status)
{
    localiz = m_loc_timeout_counter;
    laser = m_laser_timeout_counter;
    inner_status = m_inner_status_timeout_counter;
}
