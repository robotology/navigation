/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * \section robotGoto
 * robotGoto is a local navigation module. It receives a position cartesian target, either absolute (respect to the map origin) or relative (respect to the robot frame) and computes 
 * the cartesian velocity commands to be sent to baseControl module.
 * The module can be configured to stop the navigation if an obstacle is detected, or perform obstacle avoidance using an
 * APF-based approach.
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include "robotGotoDev.h"
#include <math.h>
#include <cmath>

using namespace yarp::dev::Nav2D;


YARP_LOG_COMPONENT(GOTO_DEV, "navigation.devices.robotGoto.dev")

void robotGotoRPCHandler::setInterface(robotGotoDev* iface)
{
    this->interface = iface;
}

bool robotGotoDev :: open(yarp::os::Searchable& config)
{
    string tmp_gt= config.toString();
    yCDebug(GOTO_DEV) << "RobotGoto configuration:" << tmp_gt;
    Property p; p.fromString(config.toString());

    Bottle general_group = p.findGroup("ROBOTGOTO_GENERAL");
    if (general_group.isNull())
    {
        yCError(GOTO_DEV) << "Missing ROBOTGOTO_GENERAL group!";
        return false;
    }
    if (general_group.check("name")) m_name = general_group.find("name").asString();

    //the control thread
    gotoThread = new GotoThread(0.010, p);

    if (!gotoThread->start())
    {
        delete gotoThread;
        return false;
    }

    bool ret = rpcPort.open(m_name+"/rpc");
    if (ret == false)
    {
        yCError(GOTO_DEV) << "Unable to open module ports";
        return false;
    }

    rpcPortHandler.setInterface(this);
    rpcPort.setReader(rpcPortHandler);

    return true;
}

robotGotoDev::robotGotoDev()
{
    gotoThread=NULL;
}

//module cleanup
bool robotGotoDev:: close()
{
    rpcPort.interrupt();
    rpcPort.removeCallbackLock();
    rpcPort.close();

    //gotoThread->shutdown();
    gotoThread->stop();
    delete gotoThread;
    gotoThread=NULL;

    return true;
}

bool robotGotoDev  ::parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).isString() && command.get(0).asString() == "reset_params")
    {
        gotoThread->resetParamsToDefaultValue();
        reply.addString("params reset done");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "approach")
    {
        double dir    = command.get(1).asFloat64();
        double speed  = command.get(2).asFloat64();
        double time   = command.get(3).asFloat64();
        gotoThread->approachTarget(dir,speed,time);
        reply.addString("approach command received");
    }

    else if (command.get(0).asString() == "set")
    {
        if (command.get(1).asString() == "linear_tol")
        {
            gotoThread->m_goal_tolerance_lin = command.get(2).asFloat64();
            reply.addString("linear_tol set.");
        }
        else if (command.get(1).asString() == "angular_tol")
        {
            gotoThread->m_goal_tolerance_ang = command.get(2).asFloat64();
            reply.addString("angular_tol set.");
        }
        else if (command.get(1).asString() == "max_lin_speed")
        {
            gotoThread->m_max_lin_speed = command.get(2).asFloat64();
            reply.addString("max_lin_speed set.");
        }
        else if (command.get(1).asString() == "max_ang_speed")
        {
            gotoThread->m_max_ang_speed = command.get(2).asFloat64();
            reply.addString("max_ang_speed set.");
        }
        else if (command.get(1).asString() == "min_lin_speed")
        {
            gotoThread->m_min_lin_speed = command.get(2).asFloat64();
            reply.addString("min_lin_speed set.");
        }
        else if (command.get(1).asString() == "min_ang_speed")
        {
            gotoThread->m_min_ang_speed = command.get(2).asFloat64();
            reply.addString("min_ang_speed set.");
        }
        else if (command.get(1).asString() == "ang_speed_gain")
        {
            gotoThread->m_gain_ang = command.get(2).asFloat64();
            reply.addString("ang_speed_gain set.");
        }
        else if (command.get(1).asString() == "lin_speed_gain")
        {
            gotoThread->m_gain_lin = command.get(2).asFloat64();
            reply.addString("lin_speed_gain set.");
        }
        else if (command.get(1).asString() == "obstacle_avoidance")
        {
            if (command.get(2).asInt32() == 0)
            {
                reply.addString("enable_obstacles_avoidance=false");
                gotoThread->m_enable_obstacles_avoidance = false;
            }
            else
            {
                gotoThread->m_enable_obstacles_avoidance = true;
                reply.addString("enable_obstacles_avoidance=true");
            }
        }
        else if (command.get(1).asString() == "obstacle_stop")
        {
            if (command.get(2).asInt32()==0)
            {
                reply.addString("enable_obstacle_stop=false");
                gotoThread->m_enable_obstacles_emergency_stop = false;
            }
            else
            {
                gotoThread->m_enable_obstacles_emergency_stop = true;
                reply.addString("enable_obstacle_stop=true");
            }
        }
        else
        {
            reply.addString("Unknown set.");
        }
    }
    else if (command.get(0).asString() == "get")
    {
        if (command.get(1).asString() == "navigation_status")
        {
            string s = gotoThread->getNavigationStatusAsString();
            reply.addString(s.c_str());
        }
        else
        {
            reply.addString("Unknown get.");
        }
    }
    else
    {
        reply.addString("Unknown command.");
    }
    return true;
}

ReturnValue robotGotoDev::gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc)
{
    yarp::sig::Vector v;
    v.push_back(loc.x);
    v.push_back(loc.y);
    if (std::isnan(loc.theta)==false)
    {
        v.push_back(loc.theta);
    }
    gotoThread->setNewAbsTarget(v);
    return ReturnValue_ok;
}

ReturnValue robotGotoDev::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    v.push_back(theta);
    gotoThread->setNewRelTarget(v);
    return ReturnValue_ok;
}

ReturnValue robotGotoDev::gotoTargetByRelativeLocation(double x, double y)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    gotoThread->setNewRelTarget(v);
    return ReturnValue_ok;
}

ReturnValue robotGotoDev::followPath(const yarp::dev::Nav2D::Map2DPath& path)
{
    yCError(GOTO_DEV) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue robotGotoDev::stopNavigation()
{
    bool b=gotoThread->stopMovement();
    if (b) return ReturnValue_ok;
    return ReturnValue::return_code::return_value_error_method_failed;
}

ReturnValue robotGotoDev::suspendNavigation(double time)
{
    bool b= gotoThread->pauseMovement(time);
    if (b) return ReturnValue_ok;
    return ReturnValue::return_code::return_value_error_method_failed;
}

ReturnValue robotGotoDev::resumeNavigation()
{
    bool b = gotoThread->resumeMovement();
    if (b) return ReturnValue_ok;
    return ReturnValue::return_code::return_value_error_method_failed;
}

ReturnValue robotGotoDev::getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints)
{
    yCError(GOTO_DEV) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue robotGotoDev::getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint)
{
    yCError(GOTO_DEV) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;

}

ReturnValue robotGotoDev::getCurrentNavigationMap(NavigationMapTypeEnum map_type, MapGrid2D& map)
{
    yCError(GOTO_DEV) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue robotGotoDev::getNavigationStatus(NavigationStatusEnum& status)
{
    int nav_status = gotoThread->getNavigationStatusAsInt();
    status = (NavigationStatusEnum)(nav_status);
    return ReturnValue_ok;
}

//This function parses the user commands received through the RPC port
bool robotGotoRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();

    interface->gotoThread->m_mutex.wait();

    if (command.get(0).asString() == "help")
    {
        reply.addVocab32(Vocab32::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("approach <angle in degrees> <linear velocity> <time>");
        reply.addString("reset_params");
        reply.addString("set linear_tol <m>");
        reply.addString("set linear_ang <deg>");
        reply.addString("set max_lin_speed <m/s>");
        reply.addString("set max_ang_speed <deg/s>");
        reply.addString("set min_lin_speed <m/s>");
        reply.addString("set min_ang_speed <deg/s>");
        reply.addString("set obstacle_stop <0/1>");
        reply.addString("set obstacle_avoidance <0/1>");
    }
    else if (command.get(0).isString())
    {
        interface->parse_respond_string(command, reply);
    }
    else
    {
        yCError(GOTO_DEV) << "RobotGotoDev: Received invalid command type on RPC port";
        reply.addVocab32(VOCAB_ERR);
    }

    interface->gotoThread->m_mutex.post();
    return true;
}

ReturnValue robotGotoDev::getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target)
{
    bool b= gotoThread->getCurrentAbsTarget(target);
    if (b) return ReturnValue_ok;
    return ReturnValue::return_code::return_value_error_method_failed;
}

ReturnValue robotGotoDev::recomputeCurrentNavigationPath()
{
    yCWarning(GOTO_DEV) << "robotGotoDev is not a navigation planner. recomputeCurrentNavigationPath() is not implemented.";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue robotGotoDev::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    Map2DLocation loc;
    bool b = gotoThread->getCurrentRelTarget(loc);
    x = loc.x;
    y = loc.y;
    theta = loc.theta;
    if (b) return ReturnValue_ok;
    return ReturnValue::return_code::return_value_error_method_failed;
}
