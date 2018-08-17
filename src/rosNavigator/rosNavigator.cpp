/* 
* Copyright(C)2018 ICub Facility - Istituto Italiano di Tecnologia
* Author: Marco Randazzo
* email : marco.randazzo@iit.it
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU General
* Public License for more details
*/

#define _USE_MATH_DEFINES

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include <cmath>
#include <yarp/math/Math.h>
#include "rosNavigator.h"

using namespace yarp::os;

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

rosNavigator::rosNavigator() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status = yarp::dev::navigation_status_idle;
    //### TO BE IMPLEMENTED BY USER
}

bool rosNavigator::open(yarp::os::Searchable& config)
{
    Bottle &rosGroup = config.findGroup("ROS");
    if (rosGroup.isNull())
    {
        yError() << "rosNavigator: 'ROS' group params is not a valid group or empty";
        return false;
    }

    // check for ROS_nodeName parameter
    if (!rosGroup.check("ROS_nodeName"))
    {
        yError() << "rosNavigator: cannot find ROS_nodeName parameter, mandatory when using ROS message";
        return false;
    }
    m_rosNodeName = rosGroup.find("ROS_nodeName").asString();  // TODO: check name is correct
    yInfo() << "rosNavigator: rosNodeName is " << m_rosNodeName;

    // check for ROS_topicName parameter
    if (!rosGroup.check("ROS_topicName"))
    {
        yError() << " rosNavigator: cannot find ROS_topicName parameter, mandatory when using ROS message";
        return false;
    }
    m_rosTopicName = rosGroup.find("ROS_topicName").asString();
    yInfo() << "rosNavigator: rosTopicName is " << m_rosTopicName;

    //open ROS stuff
    m_rosNode = new yarp::os::Node(m_rosNodeName);
    if (m_rosNode == nullptr)
    {
        yError() << " opening " << m_rosNodeName << " Node, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisherPort.topic(m_rosTopicName))
    {
        yError() << " opening " << m_rosTopicName << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }

    return true;
}

bool rosNavigator::close()
{
    if (m_rosNode != nullptr)
    {
        m_rosNode->interrupt();
        delete m_rosNode;
        m_rosNode = nullptr;
    }
    return true;
}

bool rosNavigator::threadInit()
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

void rosNavigator::threadRelease()
{
    //### TO BE IMPLEMENTED BY USER
}

void rosNavigator::run()
{
    //### TO BE IMPLEMENTED BY USER
}

bool rosNavigator::gotoTargetByAbsoluteLocation(yarp::dev::Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        yarp::rosmsg::geometry_msgs::PoseStamped& pos = m_rosPublisherPort.prepare();
        pos.clear();
        pos.header.frame_id = m_abs_frame_id;
        pos.header.seq = 0;
        pos.header.stamp.sec = 0;
        pos.header.stamp.nsec = 0;
        pos.pose.position.x = loc.x;
        pos.pose.position.y = loc.y;
        pos.pose.position.z = 0;
        yarp::math::Quaternion q;
        yarp::sig::Vector v(4);
        v[0] = 0;
        v[1] = 0;
        v[2] = 1;
        v[3] = loc.theta*DEG2RAD;
        q.fromAxisAngle(v);
        pos.pose.orientation.x = q.x();
        pos.pose.orientation.y = q.y();
        pos.pose.orientation.z = q.z();
        pos.pose.orientation.w = q.w();
        m_rosPublisherPort.write();
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::getNavigationStatus(yarp::dev::NavigationStatusEnum& status)
{
    status = m_navigation_status;
    return true;
}

bool rosNavigator::stopNavigation()
{
    //### TO BE IMPLEMENTED BY USER
    m_navigation_status = yarp::dev::navigation_status_idle;
    return true;
}

bool rosNavigator::getAbsoluteLocationOfCurrentTarget(yarp::dev::Map2DLocation& target)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    target = curr;
    return true;
}

bool rosNavigator::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    x = curr.x;
    y = curr.y;
    theta = curr.theta;
    return true;
}

bool rosNavigator::suspendNavigation(double time)
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

bool rosNavigator::resumeNavigation()
{
    if (m_navigation_status == yarp::dev::navigation_status_paused)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "Unable to resume any paused navigation task";
    return false;
}

bool rosNavigator::getAllNavigationWaypoints(std::vector<yarp::dev::Map2DLocation>& waypoints)
{
    yDebug() << "Not yet implemented";
    return false;
}

/**
* Returns the current waypoint pursued by the navigation algorithm
* @param curr_waypoint the current waypoint pursued by the navigation algorithm
* @return true/false
*/
bool rosNavigator::getCurrentNavigationWaypoint(yarp::dev::Map2DLocation& curr_waypoint)
{
    yDebug() << "Not yet implemented";
    return false;
}