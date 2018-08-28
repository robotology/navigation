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
 * \section robotPathPlanner
 * This module performs global path-planning by generating a sequence of waypoints to be tracked by a local navigation algorithm.
 *  It receives a goal from the user either via RPC command or via yarp iNavigation2D interface and it computes a sequence of waypoints which are sent one by one to a local navigator module.
 * If the local navigation module fails to reach one of these waypoints, the global navigation is aborted too. 
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "robotPathPlannerDev.h"
#include "pathPlannerCtrl.h"
#include <math.h>

robotPathPlannerDev::robotPathPlannerDev()
{
    m_plannerThread=NULL;
}

bool robotPathPlannerDev::open(yarp::os::Searchable& config)
{
#if 1
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = " robotPathPlanner";
    std::string file_name = " robotPathPlanner_cer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "robotPathPlannerDev configuration: \n" << p.toString().c_str();
#else
    Property p;
    p.fromString(config.toString());
#endif

    m_plannerThread = new PlannerThread(0.020,p);

    bool ret = m_rpcPort.open("/robotPathPlanner/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }
//    attach(rpcPort);//@@@@@@@@@
    //attachTerminal();

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

    
bool robotPathPlannerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear(); 

    interface->m_plannerThread->m_mutex.wait();
    if (command.get(0).isString())
    {
        if (command.get(0).asString()=="quit")
        {
            interface->m_plannerThread->m_mutex.post();
            return false;
        }

        else if (command.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("goto <locationName>");
            reply.addString("gotoAbs <x> <y> <angle in degrees>");
            reply.addString("gotoRel <x> <y> <angle in degrees>");
            reply.addString("store_current_location <location_name>");
            reply.addString("delete_location <location_name>");
            reply.addString("stop");
            reply.addString("pause");
            reply.addString("resume");
            reply.addString("quit");
            reply.addString("draw_locations <0/1>");
        }
        else if (command.get(0).isString())
        {
            interface->parse_respond_string(command, reply);
        }
    }
    else
    {
        yError() << "Invalid command type";
        reply.addVocab(VOCAB_ERR);
    }
    interface->m_plannerThread->m_mutex.post();
    return true;
}


bool robotPathPlannerDev::gotoTargetByAbsoluteLocation(yarp::dev::Map2DLocation loc)
{
    bool b = m_plannerThread->setNewAbsTarget(loc);
    return b;
}

bool robotPathPlannerDev::gotoTargetByRelativeLocation(double x, double y, double theta)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    v.push_back(theta);
    bool b = m_plannerThread->setNewRelTarget(v);
    return b;
}

bool robotPathPlannerDev::gotoTargetByRelativeLocation(double x, double y)
{
    yarp::sig::Vector v;
    v.push_back(x);
    v.push_back(y);
    bool b = m_plannerThread->setNewRelTarget(v);
    return b;
}

bool robotPathPlannerDev::getAbsoluteLocationOfCurrentTarget(yarp::dev::Map2DLocation& target)
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

bool robotPathPlannerDev::getNavigationStatus(yarp::dev::NavigationStatusEnum& status)
{
     int nav_status = m_plannerThread->getNavigationStatusAsInt();
     status = (yarp::dev::NavigationStatusEnum)(nav_status);
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

bool robotPathPlannerDev::getAllNavigationWaypoints(std::vector<yarp::dev::Map2DLocation>& waypoints)
{
    bool b = m_plannerThread->getCurrentPath(waypoints);
    return b;
}

bool robotPathPlannerDev::getCurrentNavigationWaypoint(yarp::dev::Map2DLocation& curr_waypoint)
{
    bool b = m_plannerThread->getCurrentWaypoint(curr_waypoint);
    return b;
}

bool robotPathPlannerDev::getCurrentNavigationMap(yarp::dev::NavigationMapTypeEnum map_type, yarp::dev::MapGrid2D& map)
{
    if (map_type == yarp::dev::NavigationMapTypeEnum::global_map)
    {
        m_plannerThread->getCurrentMap(map);
        return true;
    }
    else if (map_type == yarp::dev::NavigationMapTypeEnum::local_map)
    {
        return true;
    }
    return false;
}

bool robotPathPlannerDev::parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    if (command.get(0).isString() && command.get(0).asString() == "gotoAbs")
    {
        yarp::dev::Map2DLocation loc;
        loc.x = command.get(1).asDouble();
        loc.y = command.get(2).asDouble();
        if (command.size() == 4) { loc.theta = command.get(3).asDouble(); }
        else { loc.theta = nan(""); }
        loc.map_id = m_plannerThread->getCurrentMapId();
        m_plannerThread->setNewAbsTarget(loc);
        reply.addString("new absolute target received");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "gotoRel")
    {
        yarp::sig::Vector v;
        v.push_back(command.get(1).asDouble());
        v.push_back(command.get(2).asDouble());
        if (command.size() == 4) { v.push_back(command.get(3).asDouble()); }
        else { v.push_back(nan("")); }
        m_plannerThread->setNewRelTarget(v);
        reply.addString("new relative target received");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "goto")
    {
        std::string location_name = command.get(1).asString();
        if (m_plannerThread->gotoLocation(location_name))
        {
            reply.addString("goto done");
        }
        else
        {
            reply.addString("goto error");
        }
    }

    else if (command.get(0).isString() && command.get(0).asString() == "store_current_location")
    {
        std::string location_name = command.get(1).asString();
        m_plannerThread->storeCurrentLocation(location_name);
        reply.addString("store_current_location done");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "delete_location")
    {
        std::string location_name = command.get(1).asString();
        m_plannerThread->deleteLocation(location_name);
        reply.addString("delete_location done");
    }

    else if (command.get(0).isString() && command.get(0).asString() == "get_last_target")
    {
        string last_target;
        bool b = m_plannerThread->getLastTarget(last_target);
        if (b)
        {
            reply.addString(last_target);
        }
        else
        {
            yError() << "get_last_target failed: goto <location_name> target not found.";
            reply.addString("not found");
        }
    }

    else if (command.get(0).asString() == "get")
    {
        if (command.get(1).asString() == "navigation_status")
        {
            string s = m_plannerThread->getNavigationStatusAsString();
            reply.addString(s.c_str());
        }
    }
    else if (command.get(0).isString() && command.get(0).asString() == "stop")

    {
        m_plannerThread->stopMovement();
        reply.addString("Stopping movement.");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "pause")
    {
        double time = -1;
        if (command.size() > 1)
            time = command.get(1).asDouble();
        m_plannerThread->pauseMovement(time);
        reply.addString("Pausing.");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "resume")
    {
        m_plannerThread->resumeMovement();
        reply.addString("Resuming.");
    }
    else if (command.get(0).isString() && command.get(0).asString() == "draw_locations")
    {
        if (command.get(1).asInt() == 1)
        {
            m_plannerThread->m_enable_draw_all_locations = true;
            yDebug() << "locations drawing enabled";
        }
        else
        {
            m_plannerThread->m_enable_draw_all_locations = false;
            yDebug() << "locations drawing disabled";
        }
    }
    else if (command.get(0).isString() && command.get(0).asString() == "draw_enlarged_scans")
    {
        if (command.get(1).asInt() == 1)
        {
            m_plannerThread->m_enable_draw_enlarged_scans = true;
            yDebug() << "enlarged scans drawing enabled";
        }
        else
        {
            m_plannerThread->m_enable_draw_enlarged_scans = false;
            yDebug() << "enlarged scans drawing disabled";
        }
    }
    else
    {
        reply.addString("Unknown command.");
    }
    return true;
}
