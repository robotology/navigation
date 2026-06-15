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

ReturnValue robotGotoRPCHandler::ResetParams()
{
    this->interface->gotoThread->resetParamsToDefaultValue();
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::Approach(const double dir, const double speed, const double time)
{
    this->interface->gotoThread->approachTarget(dir,speed,time);
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetLinearTolerance(const double tol)
{
    this->interface->gotoThread->m_goal_tolerance_lin = tol;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetAngularTolerance(const double tol)
{
    this->interface->gotoThread->m_goal_tolerance_ang = tol;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetMaxLinearSpeed(const double maxLin)
{
    this->interface->gotoThread->m_max_lin_speed = maxLin;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetMaxAngularSpeed(const double maxAng)
{
    this->interface->gotoThread->m_max_ang_speed = maxAng;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetMinLinearSpeed(const double minLin)
{
    this->interface->gotoThread->m_min_lin_speed = minLin;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetMinAngularSpeed(const double minAng)
{
    this->interface->gotoThread->m_min_ang_speed = minAng;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetLinearSpeedGain(const double LinGain)
{
    this->interface->gotoThread->m_gain_lin = LinGain;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetAngularSpeedGain(const double AngGain)
{
    this->interface->gotoThread->m_gain_ang = AngGain;
    return ReturnValue_ok;
}

ReturnValue robotGotoRPCHandler::SetObstacoleAvoidance(const bool enable)
{
    this->interface->gotoThread->m_enable_obstacles_avoidance = enable;
    return ReturnValue_ok;
}
ReturnValue robotGotoRPCHandler::SetObstacoleStop(const bool enable)
{
    this->interface->gotoThread->m_enable_obstacles_emergency_stop = enable;
    return ReturnValue_ok;
}

return_GetNavigationStatus robotGotoRPCHandler::GetNavigationStatus()
{
    return_GetNavigationStatus ret;
    ret.ret = ReturnValue_ok;
    ret.status = this->interface->gotoThread->getNavigationStatusAsString();
    return ret;
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
