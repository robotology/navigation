/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

void robotGotoRPCHandler::setInterface(robotGotoDev* iface)
{
    this->interface = iface;
}

bool robotGotoDev :: open(yarp::os::Searchable& config)
{
#if 1

    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "robotGoto";
    std::string file_name = "robotGoto_cer.ini";
    
    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name    = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "robotGotoDev configuration: \n" << p.toString().c_str();

#else
    Property p;
    p.fromString(config.toString());
#endif
    gotoThread = new GotoThread(0.010, p);
    
    if (!gotoThread->start())
    {
        delete gotoThread;
        return false;
    }

    bool ret = rpcPort.open("/robotGoto/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
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
        double dir    = command.get(1).asDouble();
        double speed  = command.get(2).asDouble();
        double time   = command.get(3).asDouble();
        gotoThread->approachTarget(dir,speed,time);
        reply.addString("approach command received");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "gotoAbs")
    {
        yarp::sig::Vector v;
        v.push_back(command.get(1).asDouble());
        v.push_back(command.get(2).asDouble());
        if (command.size() == 4) v.push_back(command.get(3).asDouble());
        gotoThread->setNewAbsTarget(v);
        reply.addString("new absolute target received");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "gotoRel")
    {
        yarp::sig::Vector v;
        v.push_back(command.get(1).asDouble());
        v.push_back(command.get(2).asDouble());
        if (command.size() == 4) v.push_back(command.get(3).asDouble());
        gotoThread->setNewRelTarget(v);
        reply.addString("new relative target received");
    }
    else if (command.get(0).asString() == "set")
    {
        if (command.get(1).asString() == "linear_tol")
        {
            gotoThread->m_goal_tolerance_lin = command.get(2).asDouble();
            reply.addString("linear_tol set.");
        }
        else if (command.get(1).asString() == "angular_tol")
        {
            gotoThread->m_goal_tolerance_ang = command.get(2).asDouble();
            reply.addString("angular_tol set.");
        }
        else if (command.get(1).asString() == "max_lin_speed")
        {
            gotoThread->m_max_lin_speed = command.get(2).asDouble();
            reply.addString("max_lin_speed set.");
        }
        else if (command.get(1).asString() == "max_ang_speed")
        {
            gotoThread->m_max_ang_speed = command.get(2).asDouble();
            reply.addString("max_ang_speed set.");
        }
        else if (command.get(1).asString() == "min_lin_speed")
        {
            gotoThread->m_min_lin_speed = command.get(2).asDouble();
            reply.addString("min_lin_speed set.");
        }
        else if (command.get(1).asString() == "min_ang_speed")
        {
            gotoThread->m_min_ang_speed = command.get(2).asDouble();
            reply.addString("min_ang_speed set.");
        }
        else if (command.get(1).asString() == "ang_speed_gain")
        {
            gotoThread->m_gain_ang = command.get(2).asDouble();
            reply.addString("ang_speed_gain set.");
        }
        else if (command.get(1).asString() == "lin_speed_gain")
        {
            gotoThread->m_gain_lin = command.get(2).asDouble();
            reply.addString("lin_speed_gain set.");
        }
        else if (command.get(1).asString() == "obstacle_avoidance")
        {
            if (gotoThread->m_enable_obstacles_avoidance)
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
            if (gotoThread->m_enable_obstacles_emergency_stop)
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
    else if (command.get(0).isString() && command.get(0).asString() == "stop")
    {
        gotoThread->stopMovement();
        reply.addString("Stopping movement.");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "pause")
    {
        double time = -1;
        if (command.size() > 1)
            time = command.get(1).asDouble();
        gotoThread->pauseMovement(time);
        reply.addString("Pausing.");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "resume")
    {
        gotoThread->resumeMovement();
        reply.addString("Resuming.");
    }
    else
    {
        reply.addString("Unknown command.");
    }
    return true;
}

bool robotGotoDev::gotoTargetByAbsoluteLocation(yarp::dev::Map2DLocation loc)
{
    yarp::sig::Vector v;
    v.push_back(loc.x);
    v.push_back(loc.y);
    if (loc.theta != std::nan("")) v.push_back(loc.theta); //@@@check this
    bool b = true;
    gotoThread->setNewAbsTarget(v);
    return b;
}

bool robotGotoDev::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    v.push_back(theta);
    bool b = true;
    gotoThread->setNewRelTarget(v);
    return b;
}

bool robotGotoDev::gotoTargetByRelativeLocation(double x, double y)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    bool b = true;
    gotoThread->setNewRelTarget(v);
    return b;
}

bool robotGotoDev::stopNavigation()
{
    bool b=gotoThread->stopMovement();
    return b;
}

bool robotGotoDev::suspendNavigation(double time)
{
    bool b= gotoThread->pauseMovement(time);
    return b;
}

bool robotGotoDev::resumeNavigation()
{
    bool b = gotoThread->resumeMovement();
    return b;
}

bool robotGotoDev::getAllNavigationWaypoints(std::vector<yarp::dev::Map2DLocation>& waypoints)
{
    yError() << "Not yet implemented";
    return false;
}

bool robotGotoDev::getCurrentNavigationWaypoint(yarp::dev::Map2DLocation& curr_waypoint)
{
    yError() << "Not yet implemented";
    return false;
}

bool robotGotoDev::getCurrentNavigationMap(yarp::dev::NavigationMapTypeEnum map_type, yarp::dev::MapGrid2D& map)
{
    yError() << "Not yet implemented";
    return false;
}

bool robotGotoDev::getNavigationStatus(yarp::dev::NavigationStatusEnum& status)
{
    int nav_status = gotoThread->getNavigationStatusAsInt();
    status = (yarp::dev::NavigationStatusEnum)(nav_status);
    return true;
}

//This function parses the user commands received through the RPC port
bool robotGotoRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();

    interface->gotoThread->m_mutex.wait();
    if (command.get(0).asString() == "quit")
    {
        interface->gotoThread->m_mutex.post();
        return false;
    }

    else if (command.get(0).asString() == "help")
    {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("gotoAbs <x> <y> <angle in degrees>");
        reply.addString("gotoRel <x> <y> <angle in degrees>");
        reply.addString("approach <angle in degrees> <linear velocity> <time>");
        reply.addString("stop");
        reply.addString("pause");
        reply.addString("resume");
        reply.addString("quit");
        reply.addString("reset_params");
        reply.addString("set linear_tol <m>");
        reply.addString("set linear_ang <deg>");
        reply.addString("set max_lin_speed <m/s>");
        reply.addString("set max_ang_speed <deg/s>");
        reply.addString("set min_lin_speed <m/s>");
        reply.addString("set min_ang_speed <deg/s>");
        reply.addString("set obstacle_stop");
        reply.addString("set obstacle_avoidance");
    }
    else if (command.get(0).isString())
    {
        interface->parse_respond_string(command, reply);
    }
    else
    {
        yError() << "RobotGotoDev: Received invalid command type on RPC port";
        reply.addVocab(VOCAB_ERR);
    }

    interface->gotoThread->m_mutex.post();
    return true;
}

bool robotGotoDev::getAbsoluteLocationOfCurrentTarget(yarp::dev::Map2DLocation& target)
{
    bool b= gotoThread->getCurrentAbsTarget(target);
    return b;
}

bool robotGotoDev::getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta)
{
    yarp::dev::Map2DLocation loc;
    bool b = gotoThread->getCurrentRelTarget(loc);
    x = loc.x;
    y = loc.y;
    theta = loc.theta;
    return b;
}

/*
if (request == VOCAB_NAV_GET_CURRENT_POS)
{
    Map2DLocation position;
    gotoThread->getCurrentPos(position);
    reply.addVocab(VOCAB_OK);
    reply.addString(position.map_id);
    reply.addDouble(position.x);
    reply.addDouble(position.y);
    reply.addDouble(position.theta);
}
*/