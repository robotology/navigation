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
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>
#include <mutex>
#include "gazeboLocalizer.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

void gazeboLocalizerRPCHandler::setInterface(gazeboLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool gazeboLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   gazeboLocalizer::getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status)
{
    status = yarp::dev::Nav2D::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   gazeboLocalizer::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    poses.clear();
    Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return true;
}

bool   gazeboLocalizer::getCurrentPosition(Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool  gazeboLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    yError() << " gazeboLocalizer::getEstimatedOdometry is not yet implemented";
    return false;
}

bool   gazeboLocalizer::setInitialPose(const Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return true;
}

//////////////////////////

gazeboLocalizerThread::gazeboLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = 0;
    m_localization_data.y = 0;
    m_localization_data.theta = 0;

    m_local_name_prefix = "/gazeboLocalizer";
}

void gazeboLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
        yDebug() << "gazeboLocalizerThread running with period: " << this->getPeriod();
    }

    lock_guard<std::mutex> lock(m_mutex);

    //@@@@READ DATA FROM DEVICE here
    Bottle cmd, ans;
    cmd.addString("getPose");
    cmd.addString(m_object_name);
    bool ret = m_port_gazebo_comm.write(cmd, ans);
    if (ret)
    {
        if (ans.size() > 0)
        {
            //this is the full 6D pose...
            double x = ans.get(0).asFloat64();
            double y = ans.get(1).asFloat64();
            double z = ans.get(2).asFloat64();
            double ar = ans.get(3).asFloat64(); //radians
            double ap = ans.get(4).asFloat64(); //radians
            double ay = ans.get(5).asFloat64(); //radians
            //...but we are interested only into its projection on the XY map plane
            m_gazebo_data.x = x;
            m_gazebo_data.y = y;
            m_gazebo_data.theta = ay; //radians
        }
    }

    //@@@@COMPUTE LOCALIZATION DATA here
    double angle_rad = m_map_to_gazebo_transform.theta;
    m_localization_data.x     = m_map_to_gazebo_transform.x + cos (angle_rad) * m_gazebo_data.x - sin (angle_rad) * m_gazebo_data.y;
    m_localization_data.y     = m_map_to_gazebo_transform.y + sin (angle_rad) * m_gazebo_data.x + cos (angle_rad) * m_gazebo_data.y;
    m_localization_data.theta = m_gazebo_data.theta*RAD2DEG + m_map_to_gazebo_transform.theta;
    if      (m_localization_data.theta >= +360) m_localization_data.theta -= 360;
    else if (m_localization_data.theta <= -360) m_localization_data.theta += 360;
}

bool gazeboLocalizerThread::initializeLocalization(const Map2DLocation& loc)
{
    yInfo() << "gazeboLocalizerThread: Localization init request: (" << loc.map_id << ")";
    lock_guard<std::mutex> lock(m_mutex);
    //@@@@ put some check here on validity of loc
    m_localization_data.map_id = loc.map_id;
    m_map_to_gazebo_transform.map_id = loc.map_id;
    m_map_to_gazebo_transform.x = loc.x;
    m_map_to_gazebo_transform.y = loc.y;
    m_map_to_gazebo_transform.theta = loc.theta;

    return true;
}

bool gazeboLocalizerThread::getCurrentLoc(Map2DLocation& loc)
{
    lock_guard<std::mutex> lock(m_mutex);
    loc = m_localization_data;
    return true;
}

bool gazeboLocalizerThread::open_gazebo()
{
    if (m_port_gazebo_comm.open(m_local_gazebo_port_name))
    {
        if (yarp::os::Network::connect(m_local_gazebo_port_name, m_remote_gazebo_port_name, "tcp"))
        {
            return true;
        }
        yError() << "open_gazebo() failed, unable to connect port " << m_local_gazebo_port_name << " with " << m_remote_gazebo_port_name;
        return false;
    }
    yError() << "open_gazebo() failed, unable to open port " << m_local_gazebo_port_name;
    return false;
}

bool gazeboLocalizerThread::threadInit()
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

    //general group
    m_local_name_prefix = "/gazeboLocalizer";
    if (general_group.check("local_name"))
    {
        m_local_name_prefix = general_group.find("local_name").asString();
    }
    else
    {
        yInfo() << "local_name parameter not set. Using:" << m_local_name_prefix;
    }
    m_local_gazebo_port_name = m_local_name_prefix + "/gazebo_rpc";

    if (general_group.check("robot_name")) { m_object_name = general_group.find("robot_name").asString(); }
    else { yError() << "missing robot_name param"; yError() << "I need the name of the object to be localized!"; return false; }

    if (general_group.check("world_interface_port")) { m_remote_gazebo_port_name = general_group.find("world_interface_port").asString(); }
    else { yError() << "missing world_interface_port param"; return false; }

    //starts communication with gazebo
    if (!open_gazebo())
    {
        yError() << "Unable to start communication with gazebo!";
        return false;
    }

    //initial location initialization
    Map2DLocation tmp_loc;
    if (initial_group.check("map_transform_x")) { tmp_loc.x = initial_group.find("map_transform_x").asDouble(); }
    else { yError() << "missing map_transform_x param"; return false; }
    if (initial_group.check("map_transform_y")) { tmp_loc.y = initial_group.find("map_transform_y").asDouble(); }
    else { yError() << "missing map_transform_y param"; return false; }
    if (initial_group.check("map_transform_t")) { tmp_loc.theta = initial_group.find("map_transform_t").asDouble(); }
    else { yError() << "missing map_transform_t param"; return false; }
    if (initial_group.check("initial_map")) { tmp_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yError() << "missing initial_map param"; return false; }
    this->initializeLocalization(tmp_loc);

   return true;
}

void gazeboLocalizerThread::threadRelease()
{
    yDebug() << "Closing ports";
    m_port_gazebo_comm.interrupt();
    m_port_gazebo_comm.close();
}

bool gazeboLocalizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "gazeboLocalizer";
    std::string file_name = "gazeboLocalizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "gazeboLocalizer configuration: \n" << p.toString().c_str();

    std::string local_name = "gazeboLocalizer";
    Bottle general_group = p.findGroup("GENERAL");
    double period = 0.010;
    if (general_group.isNull() == false)
    {
        if (general_group.check("local_name")) { local_name = general_group.find("local_name").asString(); }
        if (general_group.check("period")) { period = general_group.find("period").asDouble(); }
    }

    thread = new gazeboLocalizerThread(period, p);

    if (!thread->start())
    {
        delete thread;
        thread = NULL;
        return false;
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

gazeboLocalizer::gazeboLocalizer()
{
    thread = NULL;
}

gazeboLocalizer::~gazeboLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool gazeboLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    yDebug() << "gazeboLocalizer terminated";
    return true;
}
 
bool   gazeboLocalizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by gazeboLocalizer";
    thread->getCurrentLoc(loc);
    return true;
}

bool   gazeboLocalizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by gazeboLocalizer";
    thread->initializeLocalization(loc);
    return true;
}

bool    gazeboLocalizer::startLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}

bool    gazeboLocalizer::stopLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}
