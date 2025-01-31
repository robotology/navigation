/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/dev/INavigation2D.h>
#include <navigation_defines.h>
#include "velocityInputHandler.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(VEL_INPUT_HANDLER, "navigation.VelocityInputHandler")

VelocityInputHandler::VelocityInputHandler()
{
    m_localName = "VelocityInputHandler_defaultName";
}

bool VelocityInputHandler::open(yarp::os::Searchable& config)
{
    if (config.check("max_timeout"))
    {
        m_max_timeout = config.find("max_timeout").asFloat64();
    }
    if (config.check("local"))
    {
        m_localName = config.find("local").asString();
    }
    return true;
}

bool VelocityInputHandler::close()
{
    return true;
}

ReturnValue VelocityInputHandler::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_control_out.linear_xvel = x_vel;
    m_control_out.linear_yvel = y_vel;
    m_control_out.angular_vel = theta_vel;
    m_control_out.timeout = timeout;
    m_control_out.reception_time = yarp::os::Time::now();
    timeout_printable = true;
    return ReturnValue_ok;
}

ReturnValue VelocityInputHandler::getLastVelocityCommand(double& x_vel, double& y_vel, double& theta_vel)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    double current_time = yarp::os::Time::now();
    if (current_time - m_control_out.reception_time > m_max_timeout)
    {
        if (timeout_printable)
        {
            yCWarning(VEL_INPUT_HANDLER) << "[" << m_localName << "] timeout:" << current_time-m_control_out.reception_time;
            timeout_printable = false;
        }
        return ReturnValue::return_code::return_value_error_method_failed;
    }
    x_vel = m_control_out.linear_xvel;
    y_vel = m_control_out.linear_yvel;
    theta_vel = m_control_out.angular_vel;
    return ReturnValue_ok;
}
