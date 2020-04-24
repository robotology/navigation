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
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <math.h>
#include "isaacLocalizer.h"

using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

void isaacLocalizerRPCHandler::setInterface(isaacLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool isaacLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   isaacLocalizer::getLocalizationStatus(LocalizationStatusEnum& status)
{
    status = LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   isaacLocalizer::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    return true;
}

bool   isaacLocalizer::getCurrentPosition(Map2DLocation& loc)
{
    if (thread)
    {
        return thread->getCurrentLoc(loc);
    }
    yError() << "isaacLocalizer thread not running";
    return false;
}

bool   isaacLocalizer::setInitialPose(const Map2DLocation& loc)
{
    yError() << "isaacLocalizer setInitialPose not yet implemented";
    return false;
}

bool isaacLocalizer::startLocalizationService()
{
    yError() << "isaacLocalizer startLocalizationService not yet implemented";
    return false;
}

bool isaacLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    yError() << "isaacLocalizer getEstimatedOdometry not yet implemented";
    return false;
}

bool isaacLocalizer::getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    if (thread)
    {
        yWarning() << "isaacLocalizer covariance matrix not set";
        thread->getCurrentLoc(loc);
        return true;
    }
    yError() << "isaacLocalizer thread not running";
    return false;
}

bool isaacLocalizer::setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yError() << "isaacLocalizer setInitialPose not yet implemented";
    return false;
}

bool isaacLocalizer::stopLocalizationService()
{
    yError() << "isaacLocalizer stopLocalizationService not yet implemented";
    return false;
}

//////////////////////////

isaacLocalizerThread::isaacLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_module_name = "localizationServer";
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = nan("");
    m_localization_data.y = nan("");
    m_localization_data.theta = nan("");
}

void isaacLocalizerThread::publish_map()
{
    //@@@
}

void isaacLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();
    
    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    yarp::sig::Vector iv;
    yarp::sig::Vector pose;
    iv.resize(6, 0.0);
    pose.resize(6, 0.0);
    
    //republish the map periodically
    if (0)
    {
        if (current_time - m_last_published_map > 5.0)
        {
            publish_map();
            m_last_published_map = yarp::os::Time::now();
        }
    }
}

bool isaacLocalizerThread::initializeLocalization(Map2DLocation& loc)
{
    m_localization_data.map_id = loc.map_id;
    
    m_localization_data.x = loc.x;
    m_localization_data.y = loc.y;
    m_localization_data.theta = loc.theta;
    
    //send data to ISAAC localization module
    //@@@
    return true;
}

bool isaacLocalizerThread::getCurrentLoc(Map2DLocation& loc)
{
    loc = m_localization_data;
    return true;
}

bool isaacLocalizerThread::threadInit()
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

    //general group
    /*if (general_group.check("module_name") == false)
    {
        yError() << "Missing `module_name` in [GENERAL] group";
        return false;
    }
    m_module_name = general_group.find("module_name").asString();
    */

    //initialize an occupancy grid publisher (every time the localization is re-initialized, the map is published too)
    //@@@

     //initialize an initial pose publisher
    //@@@

    //initial location initialization
    if (initial_group.check("initial_x"))     { m_initial_loc.x = initial_group.find("initial_x").asDouble(); }
    else { yError() << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y"))     { m_initial_loc.y = initial_group.find("initial_y").asDouble(); }
    else { yError() << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asDouble(); }
    else { yError() << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map"))   { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yError() << "missing initial_map param"; return false; }
    this->initializeLocalization(m_initial_loc);

    return true;
}

void isaacLocalizerThread::threadRelease()
{
}

bool isaacLocalizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "isaacLocalizer";
    std::string file_name = "isaacLocalizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "isaacLocalizer configuration: \n" << p.toString().c_str();

    thread = new isaacLocalizerThread(0.010, p);

    if (!thread->start())
    {
        delete thread;
        return false;
    }

    std::string local_name = "isaacLocalizer";
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

isaacLocalizer::isaacLocalizer()
{
    thread = NULL;
}

isaacLocalizer::~isaacLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool isaacLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}
