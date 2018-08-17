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
    m_rosNodeName = "rosNavigator";
    m_rosTopicName_goal = "/move_base/goal";
    m_rosTopicName_cancel = "/move_base/cancel";
    m_rosTopicName_simple_goal = "/move_base_simple/goal";
    m_rosTopicName_feedback = "/move_base/feedback";
    m_rosTopicName_status = "/move_base/status";
    m_rosTopicName_result = "/move_base/result";
    m_navigation_status = yarp::dev::navigation_status_idle;
    m_local_name_prefix = "/rosNavigator";
    m_remote_localization = "/localizationServer";
}

bool rosNavigator::open(yarp::os::Searchable& config)
{
    Bottle &rosGroup = config.findGroup("ROS");
    if (rosGroup.isNull())
    {
        //do nothing
        yInfo() << "rosNavigator: 'ROS' group param not found. Using defaults.";
    }
    else
    {
        // check for ROS_nodeName parameter
        if (!rosGroup.check("ROS_nodeName"))
        {
            yError() << "rosNavigator: cannot find ROS_nodeName parameter, mandatory when using ROS message";
            return false;
        }
        m_rosNodeName = rosGroup.find("ROS_nodeName").asString();  // TODO: check name is correct
        yInfo() << "rosNavigator: rosNodeName is " << m_rosNodeName;

        // check for ROS_topicName parameter
        if (!rosGroup.check("ROS_topicName_goal"))
        {
            yError() << " rosNavigator: cannot find ROS_topicName parameter, mandatory when using ROS message";
            return false;
        }
        m_rosTopicName_goal = rosGroup.find("ROS_topicName_goal").asString();
        yInfo() << "rosNavigator: ROS_topicName_goal is " << m_rosTopicName_goal;
    }

    //open ROS stuff
    m_rosNode = new yarp::os::Node(m_rosNodeName);
    if (m_rosNode == nullptr)
    {
        yError() << " opening " << m_rosNodeName << " Node, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_goal.topic(m_rosTopicName_goal))
    {
        yError() << " opening " << m_rosTopicName_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_cancel.topic(m_rosTopicName_cancel))
    {
        yError() << " opening " << m_rosTopicName_cancel << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_simple_goal.topic(m_rosTopicName_simple_goal))
    {
        yError() << " opening " << m_rosTopicName_simple_goal << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_feedback.topic(m_rosTopicName_feedback))
    {
        yError() << " opening " << m_rosTopicName_feedback << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_status.topic(m_rosTopicName_status))
    {
        yError() << " opening " << m_rosTopicName_status << " Topic, check your yarp-ROS network configuration\n";
        return false;
    }
    if (!m_rosPublisher_result.topic(m_rosTopicName_result))
    {
        yError() << " opening " << m_rosTopicName_result << " Topic, check your yarp-ROS network configuration\n";
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
    //localization
    Property loc_options;
    loc_options.put("device", "localization2DClient");
    loc_options.put("local", m_local_name_prefix + "/localizationClient");
    loc_options.put("remote", m_remote_localization);
    if (m_pLoc.open(loc_options) == false)
    {
        yError() << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_pLoc.isValid() == false || m_iLoc == 0)
    {
        yError() << "Unable to view localization interface";
        return false;
    }

    return true;
}

void rosNavigator::threadRelease()
{
    //### TO BE IMPLEMENTED BY USER
}

void rosNavigator::run()
{
    bool b = m_iLoc->getCurrentPosition(m_current_position);
}

bool rosNavigator::gotoTargetByAbsoluteLocation(yarp::dev::Map2DLocation loc)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        yarp::rosmsg::geometry_msgs::PoseStamped& pos = m_rosPublisher_simple_goal.prepare();
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
        m_rosPublisher_simple_goal.write();
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        yarp::dev::Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - theta; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool rosNavigator::gotoTargetByRelativeLocation(double x, double y)
{
    if (m_navigation_status == yarp::dev::navigation_status_idle)
    {
        yarp::dev::Map2DLocation loc;
        loc.map_id = m_current_position.map_id;
        loc.x = m_current_position.x - x; //@@@THIS NEEDS TO BE FIXED
        loc.y = m_current_position.y - y; //@@@THIS NEEDS TO BE FIXED
        loc.theta = m_current_position.theta - 0; //@@@THIS NEEDS TO BE FIXED
        return gotoTargetByAbsoluteLocation(loc);
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
    target = m_current_goal;
    return true;
}

bool rosNavigator::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    x = m_current_goal.x- m_current_position.x; // @@@THIS NEEDS TO BE FIXED
    y = m_current_goal.y - m_current_position.y; // @@@THIS NEEDS TO BE FIXED
    theta = m_current_goal.theta - m_current_position.theta; // @@@THIS NEEDS TO BE FIXED
    return true;
}

bool rosNavigator::suspendNavigation(double time)
{
    yError() << "Unable to pause current navigation task";
    return false;
}

bool rosNavigator::resumeNavigation()
{
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

bool rosNavigator::getCurrentNavigationMap(yarp::dev::NavigationMapTypeEnum map_type, yarp::dev::MapGrid2D& map)
{
    yDebug() << "Not yet implemented";
    return false;
}