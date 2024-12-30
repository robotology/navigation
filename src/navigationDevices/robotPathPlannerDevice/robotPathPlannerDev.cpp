/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * \section robotPathPlanner
 * This module performs global path-planning by generating a sequence of waypoints to be tracked by a local navigation algorithm.
 *  It receives a goal from the user either via RPC command or via yarp iNavigation2D interface and it computes a sequence of waypoints which are sent one by one to a local navigator module.
 * If the local navigation module fails to reach one of these waypoints, the global navigation is aborted too. 
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

#include <yarp/os/Log.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "robotPathPlannerDev.h"
#include "pathPlannerCtrl.h"
#include <math.h>

using namespace yarp::dev::Nav2D;

YARP_LOG_COMPONENT(PATHPLAN_DEV, "navigation.devices.robotPathPlanner.dev")

robotPathPlannerDev::robotPathPlannerDev()
{
    m_plannerThread=NULL;
}

bool robotPathPlannerDev::open(yarp::os::Searchable& config)
{
    //default values
    Property p; p.fromString(config.toString());

    Bottle general_group = p.findGroup("PATHPLANNER_GENERAL");
    if (general_group.isNull())
    {
        yCError(PATHPLAN_DEV) << "Missing PATHPLANNER_GENERAL group!";
        return false;
    }
    if (general_group.check("name")) m_name = general_group.find("name").asString();

    //Call the parse of `navigation_with_stuck_detection`
    if (initialize_recovery(config) == false) return false;

    //the control thread
    m_plannerThread = new PlannerThread(0.020,p);

    bool ret = m_rpcPort.open(m_name+"/rpc");
    if (ret == false)
    {
        yCError(PATHPLAN_DEV) << "Unable to open module ports";
        return false;
    }

    m_rpcPort.setReader(*this);

    if (!m_plannerThread->start())
    {
        delete m_plannerThread;
        return false;
    }

    return true;
}

bool robotPathPlannerDev::close()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();

    //gotoThread->shutdown();
    m_plannerThread->stop();
    delete m_plannerThread;
    m_plannerThread=NULL;

    return true;
}


bool robotPathPlannerDev::read(yarp::os::ConnectionReader& connection)
{
    yCDebug(PATHPLAN_DEV);
    yarp::os::Bottle command;
    yarp::os::Bottle reply;
    bool ok = command.read(connection);
    if (!ok) return false;
    reply.clear();

    m_plannerThread->m_mutex.wait();
    if (command.get(0).isString())
    {
        if (command.get(0).asString()=="help")
        {
            reply.addVocab32(Vocab32::encode("many"));
            reply.addString("set_robot_radius <size_m>");
            reply.addString("get_robot_radius");
        }
        else if (command.get(0).isString())
        {
            parse_respond_string(command, reply);
        }
    }
    else
    {
        yCError(PATHPLAN_DEV) << "Invalid command type";
        reply.addVocab32(VOCAB_ERR);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        reply.write(*returnToSender);
    }
    m_plannerThread->m_mutex.post();
    return true;
}


bool robotPathPlannerDev::gotoTargetByAbsoluteLocation(Map2DLocation loc)
{
    bool b = true;
    b &= m_plannerThread->reloadCurrentMap();
    b &= m_plannerThread->setNewAbsTarget(loc);
    m_plannerThread->resetAttemptCounter();
    return b;
}

bool robotPathPlannerDev::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    v.push_back(theta);
    bool b = true;
    b &= m_plannerThread->reloadCurrentMap();
    b &= m_plannerThread->setNewRelTarget(v);
    m_plannerThread->resetAttemptCounter();
    return b;
}

bool robotPathPlannerDev::followPath(const Map2DPath& path)
{
    yError("robotPathPlannerDev::goThroughTargetsByAbsoluteLocations() not implemented yet");
    return false;
}

bool robotPathPlannerDev::recomputeCurrentNavigationPath()
{
    bool b= m_plannerThread->recomputePath();
    if (b==false)
    {
        yCError(PATHPLAN_DEV) << "robotPathPlannerDev::recomputeCurrentNavigationPath(). An error occurred while performing the requested operation.";
        return false;
    }
    return true;
}

bool robotPathPlannerDev::gotoTargetByRelativeLocation(double x, double y)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    bool b = true;
    b &= m_plannerThread->reloadCurrentMap();
    b &= m_plannerThread->setNewRelTarget(v);
    m_plannerThread->resetAttemptCounter();
    return b;
}

bool robotPathPlannerDev::getAbsoluteLocationOfCurrentTarget(Map2DLocation& target)
{
    m_plannerThread->getCurrentAbsTarget(target);
    return true;
}

bool robotPathPlannerDev::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    Map2DLocation loc;
    m_plannerThread->getCurrentRelTarget(loc);
    x=loc.x;
    y=loc.y;
    theta=loc.theta;
    return true;
}

bool robotPathPlannerDev::getNavigationStatus(NavigationStatusEnum& status)
{
     int nav_status = m_plannerThread->getNavigationStatusAsInt();
     status = (NavigationStatusEnum)(nav_status);
     return true;
}

bool robotPathPlannerDev::stopNavigation()
{
     bool b = m_plannerThread->stopMovement();
     return true;
}

bool robotPathPlannerDev::suspendNavigation(double time)
{
     bool b = m_plannerThread->pauseMovement(time);
     return b;
}

bool robotPathPlannerDev::resumeNavigation()
{
     bool b = m_plannerThread->resumeMovement();
     return b;
}

bool robotPathPlannerDev::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints)
{
    bool b = m_plannerThread->getCurrentPath(waypoints);
    return b;
}

bool robotPathPlannerDev::getCurrentNavigationWaypoint(Map2DLocation& curr_waypoint)
{
    bool b = m_plannerThread->getCurrentWaypoint(curr_waypoint);
    return b;
}

bool robotPathPlannerDev::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D& map)
{
    if (map_type == NavigationMapTypeEnum::global_map)
    {
        m_plannerThread->getCurrentMap(map);
        return true;
    }
    else if (map_type == NavigationMapTypeEnum::local_map)
    {
        m_plannerThread->getOstaclesMap(map);
        return true;
    }
    return false;
}

bool robotPathPlannerDev::parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).asString() == "set_robot_radius")
    {
        bool ret = this->m_plannerThread->setRobotRadius(command.get(1).asFloat64());
        if (ret)
        {
            reply.addString("set_robot_radius done");
        }
        else
        {
            reply.addString("set_robot_radius failed");
        }
    }
    if (command.get(0).asString() == "get_robot_radius")
    {
        double value = 0;
        bool ret = this->m_plannerThread->getRobotRadius(value);
        if (ret)
        {
            reply.addFloat64(value);
        }
        else
        {
            reply.addString("get_robot_radius failed");
        }
    }
    return true;
}
