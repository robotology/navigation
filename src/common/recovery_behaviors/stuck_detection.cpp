/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "stuck_detection.h"

using namespace yarp::dev::Nav2D;
using namespace yarp::os;

YARP_LOG_COMPONENT(STUCK_DETECT, "navigation.navigation_lib.struck_detection")

bool navigation_with_stuck_detection::initialize_recovery(yarp::os::Searchable& config)
{
    yarp::os::Bottle& recovGroup = config.findGroup("RECOVERY");
    if (recovGroup.isNull())
    {
        yCInfo(STUCK_DETECT) << "No recovery strategy selected. If needed please add a RECOVERY section";
    }
    else
    {
        if (!recovGroup.check("enable")) {yCError(STUCK_DETECT) << "Missing RECOVERY::enable option"; return false;}
        m_stuck_recovery_enable = recovGroup.find("enable").asBool();
        if (!recovGroup.check("lin_tolerance")) { yCError(STUCK_DETECT) << "Missing RECOVERY::lin_tolerance option"; return false; }
        m_stuck_linear_tolerance = recovGroup.find("lin_tolerance").asFloat64();
        if (!recovGroup.check("ang_tolerance")) { yCError(STUCK_DETECT) << "Missing RECOVERY::ang_tolerance option"; return false; }
        m_stuck_angular_tolerance = recovGroup.find("ang_tolerance").asFloat64();
        if (!recovGroup.check("time_tolerance")) { yCError(STUCK_DETECT) << "Missing RECOVERY::time_tolerance option"; return false; }
        m_stuck_time_tolerance = recovGroup.find("time_tolerance").asFloat64();
        if (!recovGroup.check("algorithm_name")) { yCError(STUCK_DETECT) << "Missing RECOVERY::algorithm_name option"; return false; }
        m_recovery_algorithm_name = recovGroup.find("algorithm_name").asString();
        
        if (m_recovery_algorithm_name == "rotation_in_place")
        {
//            m_recovery_behavior = new recovery_behavior(&m_aux_cmd_port, nullptr, m_iLoc, m_iMap);
        }
        else
        {
            yCError(STUCK_DETECT) << "Unknown recovery algorithm:" << m_recovery_algorithm_name;
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