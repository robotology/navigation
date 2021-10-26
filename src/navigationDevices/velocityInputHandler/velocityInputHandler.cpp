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
    m_localName = "VelocityInputHandler";
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

bool VelocityInputHandler::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    m_control_out.linear_xvel = x_vel;
    m_control_out.linear_yvel = y_vel;
    m_control_out.angular_vel = theta_vel;
    m_control_out.timeout = timeout;
    m_control_out.reception_time = yarp::os::Time::now();
    return true;
}

bool VelocityInputHandler::getLastVelocityCommand(double& x_vel, double& y_vel, double& theta_vel)
{
    double current_time = yarp::os::Time::now();
    if (current_time - m_control_out.reception_time > m_max_timeout)
    {
        return false;
    }
    x_vel = m_control_out.linear_xvel;
    y_vel = m_control_out.linear_yvel;
    theta_vel = m_control_out.angular_vel;
    return true;
}
