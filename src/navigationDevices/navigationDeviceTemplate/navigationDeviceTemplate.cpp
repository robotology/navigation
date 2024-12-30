/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include "navigationDeviceTemplate.h"

using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(NAVIGATOR_TEMPLATE, "navigation.NavigatonDeviceTemplate")

navigationDeviceTemplate::navigationDeviceTemplate() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = navigation_status_idle;
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

bool navigationDeviceTemplate::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    if (m_navigation_status == navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "A navigation task is already running. Stop it first";
    return false;
}

bool navigationDeviceTemplate::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "A navigation task is already running. Stop it first";
    return false;

}
bool navigationDeviceTemplate::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "A navigation task is already running. Stop it first";
    return false;
}

bool navigationDeviceTemplate::getNavigationStatus(NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool navigationDeviceTemplate::stopNavigation()
{
    //### TO BE IMPLEMENTED BY USER
    m_navigation_status = navigation_status_idle;
    return true;
}

bool navigationDeviceTemplate::getAbsoluteLocationOfCurrentTarget(Map2DLocation& target)
{
    //### TO BE IMPLEMENTED BY USER
    Map2DLocation curr;
    target = curr;
    return true;
}

bool navigationDeviceTemplate::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    //### TO BE IMPLEMENTED BY USER
    Map2DLocation curr;
    x = curr.x;
    y = curr.y;
    theta = curr.theta;
    return true;
}

bool navigationDeviceTemplate::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::getCurrentNavigationWaypoint(Map2DLocation& curr_waypoint)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool navigationDeviceTemplate::suspendNavigation(double time)
{
    if (m_navigation_status == navigation_status_moving)
    {
        //### TO BE IMPLEMENTED BY USER
        m_navigation_status = navigation_status_paused;
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "Unable to pause current navigation task";
    return false;
}

bool navigationDeviceTemplate::resumeNavigation()
{
    if (m_navigation_status == navigation_status_paused)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "Unable to resume any paused navigation task";
    return false;
}

bool navigationDeviceTemplate::recomputeCurrentNavigationPath()
{
    if (m_navigation_status == navigation_status_moving)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yCError(NAVIGATOR_TEMPLATE) << "Unable to recompute path. Navigation task not assigned yet.";
    return false;
}

bool navigationDeviceTemplate::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D& map)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}
