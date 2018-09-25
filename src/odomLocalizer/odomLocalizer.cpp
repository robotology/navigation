/*
 * Copyright (C)2017  ICub Facility - Istituto Italiano di Tecnologia
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
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>
#include "odomLocalizer.h"

using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

void odomLocalizerRPCHandler::setInterface(odomLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool odomLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   odomLocalizer::getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status)
{
    status = yarp::dev::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   odomLocalizer::getEstimatedPoses(std::vector<yarp::dev::Map2DLocation>& poses)
{
    poses.clear();
    yarp::dev::Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return true;
}

bool   odomLocalizer::getCurrentPosition(yarp::dev::Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool   odomLocalizer::setInitialPose(yarp::dev::Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return true;
}

//////////////////////////

odomLocalizerThread::odomLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_last_odometry_data_received = -1;
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = nan("");
    m_localization_data.y = nan("");
    m_localization_data.theta = nan("");
}

void odomLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    LockGuard lock(m_mutex);
    yarp::sig::Vector *loc = m_port_odometry_input.read(false);
    if (loc)
    {
        m_last_odometry_data_received = yarp::os::Time::now();
        m_localization_data.x = loc->data()[0];
        m_localization_data.y = loc->data()[1];
        m_localization_data.theta = loc->data()[2];
    }
    if (current_time - m_last_odometry_data_received > 0.1)
    {
        yWarning() << "No localization data received for more than 0.1s!";
    }
}

bool odomLocalizerThread::initializeLocalization(yarp::dev::Map2DLocation& loc)
{
    m_localization_data.map_id = loc.map_id;
    m_localization_data.x = loc.x;
    m_localization_data.y = loc.y;
    m_localization_data.theta = loc.theta;
    return true;
}

bool odomLocalizerThread::getCurrentLoc(yarp::dev::Map2DLocation& loc)
{
    loc = m_localization_data;
    return true;
}

bool odomLocalizerThread::threadInit()
{
    //configuration file checking
    Bottle general_group = m_cfg.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError() << "Missing GENERAL group!";
        return false;
    }

    Bottle initial_group = m_cfg.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yError() << "Missing INITIAL_POS group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yError() << "Missing LOCALIZATION group!";
        return false;
    }

    Bottle tf_group = m_cfg.findGroup("TF");
    if (tf_group.isNull())
    {
        yError() << "Missing TF group!";
        return false;
    }

    Bottle odometry_group = m_cfg.findGroup("ODOMETRY");
    if (odometry_group.isNull())
    {
        yError() << "Missing ODOMETRY group!";
        return false;
    }

    //general group
    m_local_name = "odomLocalizer";
    if (general_group.check("local_name")) { m_local_name = general_group.find("local_name").asString();}

    //odometry group
    if (odometry_group.check("odometry_broadcast_port") == false)
    {
        yError() << "Missing `odometry_port` in [ODOMETRY] group";
        return false;
    }
    m_port_broadcast_odometry_name = odometry_group.find("odometry_broadcast_port").asString();

    //opens a YARP port to receive odometry data
    std::string odom_portname = "/" + m_local_name + "/odometry:i";
    bool b1 = m_port_odometry_input.open(odom_portname.c_str());
    bool b2 = yarp::os::Network::sync(odom_portname.c_str(), false);
    bool b3 = yarp::os::Network::connect(m_port_broadcast_odometry_name.c_str(), odom_portname.c_str());
    if (b1 == false || b2 == false || b3 == false)
    {
        yError() << "Unable to initialize odometry port connection from " << m_port_broadcast_odometry_name.c_str() << "to:" << odom_portname.c_str();
        return false;
    }

    //initial location initialization
    if (initial_group.check("initial_x")) { m_initial_loc.x = initial_group.find("initial_x").asDouble(); }
    else { yError() << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y")) { m_initial_loc.y = initial_group.find("initial_y").asDouble(); }
    else { yError() << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asDouble(); }
    else { yError() << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map")) { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yError() << "missing initial_map param"; return false; }
    this->initializeLocalization(m_initial_loc);

   return true;
}

void odomLocalizerThread::threadRelease()
{

}


bool odomLocalizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "odomLocalizer";
    std::string file_name = "odomLocalizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "odomLocalizer configuration: \n" << p.toString().c_str();

    thread = new odomLocalizerThread(0.010, p);

    if (!thread->start())
    {
        delete thread;
        return false;
    }

    std::string local_name = "odomLocalizer";
    Bottle general_group = p.findGroup("GENERAL");
    if (general_group.isNull()==false)
    {
        if (general_group.check("local_name")) { local_name = general_group.find("local_name").asString(); }
    }
    bool ret = rpcPort.open("/"+local_name+"/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    rpcPortHandler.setInterface(this);
    rpcPort.setReader(rpcPortHandler);

    return true;
}

odomLocalizer::odomLocalizer()
{
    thread = NULL;
}

odomLocalizer::~odomLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool odomLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}
 
