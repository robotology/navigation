/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include "stuck_detection.h"

using namespace yarp::dev::Nav2D;
using namespace yarp::os;

bool navigation_with_stuck_detection::initialize_recovery(yarp::os::Searchable& config)
{
    yarp::os::Bottle& recovGroup = config.findGroup("RECOVERY");
    if (recovGroup.isNull())
    {
        yInfo() << "No recovery strategy selected. If needed please add a RECOVERY section";
    }
    else
    {
        if (!recovGroup.check("enable")) {yError() << "Missing RECOVERY::enable option"; return false;}
        m_stuck_recovery_enable = recovGroup.find("enable").asBool();
        if (!recovGroup.check("lin_tolerance")) { yError() << "Missing RECOVERY::lin_tolerance option"; return false; }
        m_stuck_linear_tolerance = recovGroup.find("lin_tolerance").asFloat64();
        if (!recovGroup.check("ang_tolerance")) { yError() << "Missing RECOVERY::ang_tolerance option"; return false; }
        m_stuck_angular_tolerance = recovGroup.find("ang_tolerance").asFloat64();
        if (!recovGroup.check("time_tolerance")) { yError() << "Missing RECOVERY::time_tolerance option"; return false; }
        m_stuck_time_tolerance = recovGroup.find("time_tolerance").asFloat64();
        if (!recovGroup.check("algorithm_name")) { yError() << "Missing RECOVERY::algorithm_name option"; return false; }
        m_recovery_algorithm_name = recovGroup.find("algorithm_name").asString();
        
        if (m_recovery_algorithm_name == "rotation_in_place")
        {
//            m_recovery_behavior = new recovery_behavior(&m_aux_cmd_port, nullptr, m_iLoc, m_iMap);
        }
        else
        {
            yError() << "Unknown recovery algorithm:" << m_recovery_algorithm_name;
            return false;
        }
    }
    return true;
}


bool navigation_with_stuck_detection::we_are_stuck(const Map2DLocation& current_position)
{
    //yarp::dev:Nav2D::
    if (current_position.is_near_to(m_last_position, m_stuck_linear_tolerance, m_stuck_angular_tolerance))
    {
        if (yarp::os::Time::now() - m_time_suspected_stuck > m_stuck_time_tolerance)
        {
            return true;
        }
    }
    else
    {
        m_last_position = current_position;
        m_time_suspected_stuck = yarp::os::Time::now();
    }
    return false;
}