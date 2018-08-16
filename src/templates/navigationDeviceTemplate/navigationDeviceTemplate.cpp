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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include "navigationDeviceTemplate.h"

navigationDeviceTemplate::navigationDeviceTemplate() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = yarp::dev::navigation_status_idle;
    //### TO BE IMPLEMENTED BY USER
}

bool navigationDeviceTemplate::open(yarp::os::Searchable& config)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::close()
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::threadInit()
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

void navigationDeviceTemplate::threadRelease()
{
    //### TO BE IMPLEMENTED BY USER
}

void navigationDeviceTemplate::run()
{
    //### TO BE IMPLEMENTED BY USER
}

bool navigationDeviceTemplate::gotoTargetByAbsoluteLocation(yarp::dev::Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool navigationDeviceTemplate::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;

}
bool navigationDeviceTemplate::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool navigationDeviceTemplate::getNavigationStatus(yarp::dev::NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool navigationDeviceTemplate::stopNavigation()
{
    //### TO BE IMPLEMENTED BY USER
    m_navigation_status = yarp::dev::navigation_status_idle;
    return true;
}

bool navigationDeviceTemplate::getAbsoluteLocationOfCurrentTarget(yarp::dev::Map2DLocation& target)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    target = curr;
    return true;
}

bool navigationDeviceTemplate::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    x = curr.x;
    y = curr.y;
    theta = curr.theta;
    return true;
}

bool navigationDeviceTemplate::getNavigationWaypoints(std::vector<yarp::dev::Map2DLocation>& waypoints)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::getCurrentNavigationWaypoint(yarp::dev::Map2DLocation& curr_waypoint)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::suspendNavigation(double time)
{
    if (m_navigation_status == yarp::dev::navigation_status_moving)
    {
        //### TO BE IMPLEMENTED BY USER
        m_navigation_status = yarp::dev::navigation_status_paused;
        return true;
    }
    yError() << "Unable to pause current navigation task";
    return false;
}

bool navigationDeviceTemplate::resumeNavigation()
{
    if (m_navigation_status == yarp::dev::navigation_status_paused)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "Unable to resume any paused navigation task";
    return false;
}
