/*
 * Copyright (C)2018  ICub Facility - Istituto Italiano di Tecnologia
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
#include "pozyxLocalizer.h"

using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

void pozyxLocalizerRPCHandler::setInterface(pozyxLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool pozyxLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   pozyxLocalizer::getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status)
{
    status = yarp::dev::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   pozyxLocalizer::getEstimatedPoses(std::vector<yarp::dev::Map2DLocation>& poses)
{
    poses.clear();
    yarp::dev::Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return true;
}

bool   pozyxLocalizer::getCurrentPosition(yarp::dev::Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool   pozyxLocalizer::setInitialPose(yarp::dev::Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return true;
}

//////////////////////////

pozyxLocalizerThread::pozyxLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = 0;
    m_localization_data.y = 0;
    m_localization_data.theta = 0;
}

void pozyxLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    LockGuard lock(m_mutex);

    //@@@@READ DATA FROM DEVICE here
    m_pozyx_data.x=0;
    m_pozyx_data.y=0;
    m_pozyx_data.theta=0;

    //@@@@COMPUTE LOCALIZATION DATA here
    m_localization_data.x     = m_pozyx_data.x     + m_initial_loc.x;
    m_localization_data.y     = m_pozyx_data.y     + m_initial_loc.y;
    m_localization_data.theta = m_pozyx_data.theta + m_initial_loc.theta;
    if      (m_localization_data.theta >= +360) m_localization_data.theta -= 360;
    else if (m_localization_data.theta <= -360) m_localization_data.theta += 360;
}

bool pozyxLocalizerThread::initializeLocalization(yarp::dev::Map2DLocation& loc)
{
    yInfo() << "pozyxLocalizerThread: Localization init request: (" << loc.map_id << ")";
    LockGuard lock(m_mutex);
    m_initial_loc.map_id = loc.map_id;
    m_initial_loc.x = -m_pozyx_data.x + loc.x;
    m_initial_loc.y = -m_pozyx_data.y + loc.y;
    m_initial_loc.theta = -m_pozyx_data.theta + loc.theta;
    if (m_localization_data.map_id != m_initial_loc.map_id)
    {
        yInfo() << "Map changed to: " << m_localization_data.map_id;
        m_localization_data.map_id = m_initial_loc.map_id;
        //@@@TO BE COMPLETED
        m_localization_data.x = 0+m_initial_loc.x;
        m_localization_data.y = 0+m_initial_loc.y;
        m_localization_data.theta = 0+m_initial_loc.theta;
    }
    return true;
}

bool pozyxLocalizerThread::getCurrentLoc(yarp::dev::Map2DLocation& loc)
{
    LockGuard lock(m_mutex);
    loc = m_localization_data;
    return true;
}

bool pozyxLocalizerThread::threadInit()
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

    //general group
    m_local_name = "pozyxLocalizer";
    if (general_group.check("local_name")) { m_local_name = general_group.find("local_name").asString();}

    //opens the pozyx device communication
    //@@@@ TO BE IMPLEMENTED

    //initial location initialization
    yarp::dev::Map2DLocation tmp_loc;
    if (initial_group.check("initial_x")) { tmp_loc.x = initial_group.find("initial_x").asDouble(); }
    else { yError() << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y")) { tmp_loc.y = initial_group.find("initial_y").asDouble(); }
    else { yError() << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { tmp_loc.theta = initial_group.find("initial_theta").asDouble(); }
    else { yError() << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map")) { tmp_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yError() << "missing initial_map param"; return false; }
    this->initializeLocalization(tmp_loc);

   return true;
}

void pozyxLocalizerThread::threadRelease()
{

}


bool pozyxLocalizer::open(yarp::os::Searchable& config)
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

    thread = new pozyxLocalizerThread(0.010, p);

    if (!thread->start())
    {
        delete thread;
        thread = NULL;
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

pozyxLocalizer::pozyxLocalizer()
{
    thread = NULL;
}

pozyxLocalizer::~pozyxLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool pozyxLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}
 
