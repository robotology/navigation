/*
 * Copyright (C)2018 ICub Facility - Istituto Italiano di Tecnologia
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

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include <limits>
#include <navigation_defines.h>
#include "simpleVelocityNavigation.h"

//the following ugly definition is just a reminder to check the system behavior when a method returns true (with invalid data) or false.
#define NOT_IMPLEMENTED false

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(SIMPLE_VEL_NAV, "navigation.simpleVelocityNavigator")

simpleVelocityNavigation::simpleVelocityNavigation() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = navigation_status_idle;
    m_localName = "simpleVelocityNavigation";
    m_send_zero_when_expired = false;
}

bool simpleVelocityNavigation::open(yarp::os::Searchable& config)
{
    bool ret = true;
    ret &= m_port_commands_output.open((std::string("/")+m_localName + "/control:o").c_str());

    if (ret == false)
    {
        yCError(SIMPLE_VEL_NAV) << "Unable to open module ports";
        return false;
    }
    this->start();
    return ret;
}

bool simpleVelocityNavigation::close()
{
    this->askToStop();
    while(this->isRunning())
    {
        yarp::os::Time::delay(1.0); //wait until thread is stopped
    }
    m_port_commands_output.interrupt();
    m_port_commands_output.close();
    return true;
}

bool simpleVelocityNavigation::threadInit()
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

void simpleVelocityNavigation::threadRelease()
{
    //### TO BE IMPLEMENTED BY USER
}

void simpleVelocityNavigation::run()
{

    double current_time = yarp::os::Time::now();

    if (m_port_commands_output.getOutputCount() > 0)
    {
        if (m_control_out.timeout == std::numeric_limits<double>::infinity())
        {
            send_command(m_control_out);
            m_navigation_status = navigation_status_moving;
        }
        else
        {
            if ((current_time - m_control_out.reception_time) < m_control_out.timeout)
            {
                send_command(m_control_out);
                m_navigation_status = navigation_status_moving;
            }
            else
            {
                //control timeout expired
                if (m_send_zero_when_expired)
                {
                    //send a zero command
                    send_command(control_type());
                }
                else
                {
                    //do not send anything
                }
                m_navigation_status = navigation_status_idle;
            }
        }
    }
}

void simpleVelocityNavigation::send_command(control_type control_data)
{
    static yarp::os::Stamp stamp;
    stamp.update();
    Bottle &b = m_port_commands_output.prepare();
    m_port_commands_output.setEnvelope(stamp);
    b.clear();
    b.addInt(BASECONTROL_COMMAND_VELOCIY_CARTESIAN);
    b.addDouble(control_data.linear_xvel);    // lin_vel in m/s
    b.addDouble(control_data.linear_yvel);    // lin_vel in m/s
    b.addDouble(control_data.angular_vel);    // ang_vel in deg/s
    b.addDouble(100);
    m_port_commands_output.write();
}

bool simpleVelocityNavigation::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    yCError(SIMPLE_VEL_NAV) << "gotoTargetByAbsoluteLocation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::gotoTargetByRelativeLocation(double x, double y)
{
    yCError(SIMPLE_VEL_NAV) << "gotoTargetByRelativeLocation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    yCError(SIMPLE_VEL_NAV) << "gotoTargetByRelativeLocation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout)
{
    m_control_out.linear_xvel = x_vel;
    m_control_out.linear_yvel = y_vel;
    m_control_out.angular_vel = theta_vel;
    m_control_out.timeout = timeout;
    m_control_out.reception_time = yarp::os::Time::now();
    return true;
}

bool simpleVelocityNavigation::getNavigationStatus(NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool simpleVelocityNavigation::stopNavigation()
{
    yCError(SIMPLE_VEL_NAV) << "stopNavigation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::getAbsoluteLocationOfCurrentTarget(Map2DLocation& target)
{
    yCError(SIMPLE_VEL_NAV) << "getAbsoluteLocationOfCurrentTarget() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    yCError(SIMPLE_VEL_NAV) << "getRelativeLocationOfCurrentTarget() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints)
{
    yCError(SIMPLE_VEL_NAV) << "getAllNavigationWaypoints() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::getCurrentNavigationWaypoint(Map2DLocation& curr_waypoint)
{
    yCError(SIMPLE_VEL_NAV) << "getCurrentNavigationWaypoint() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::suspendNavigation(double time)
{
    yCError(SIMPLE_VEL_NAV) << "suspendNavigation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::resumeNavigation()
{
    yCError(SIMPLE_VEL_NAV) << "resumeNavigation() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::recomputeCurrentNavigationPath()
{
    yCError(SIMPLE_VEL_NAV) << "recomputeCurrentNavigationPath() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}

bool simpleVelocityNavigation::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D& map)
{
    yCError(SIMPLE_VEL_NAV) << "getCurrentNavigationMap() Not implemented by simpleVelocityNavigation";
    return NOT_IMPLEMENTED;
}