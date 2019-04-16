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
using namespace isaac;

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


bool   isaacLocalizer::getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status)
{
    status = yarp::dev::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   isaacLocalizer::getEstimatedPoses(std::vector<yarp::dev::Map2DLocation>& poses)
{
    return true;
}

bool   isaacLocalizer::getCurrentPosition(yarp::dev::Map2DLocation& loc)
{
	if (thread)
	{
		return thread->getCurrentLoc(loc);
	}
	yError() << "isaacLocalizer thread not running";
    return false;
}

bool   isaacLocalizer::setInitialPose(yarp::dev::Map2DLocation& loc)
{
	if (thread)
	{
		return thread->initializeLocalization(loc);
	}
	yError() << "isaacLocalizer thread not running";
    return false;
}

//////////////////////////

isaacLocalizerThread::isaacLocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_iMap = 0;
    m_iTf = 0;
    m_ros_enabled = false;
    m_module_name = "localizationServer";
    m_tf_data_received = -1;
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
/////////////////////////////////////////////////////////

  
    bool ok;
    const Pose2d world_T_robot = get_world_T_robot(getTickTime(), ok);
    if (ok)
    {
      double x = world_T_robot.translation(0);
      double y = world_T_robot.translation(1);
      double t = world_T_robot.rotation.angle();
    }


/////////////////////////////////////////////////////////
    double current_time = yarp::os::Time::now();
    
    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    LockGuard lock(m_mutex);
    yarp::sig::Vector iv;
    yarp::sig::Vector pose;
    iv.resize(6, 0.0);
    pose.resize(6, 0.0);
    bool r = m_iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);
    if (r)
    {
        //data is formatted as follows: x, y, angle (in degrees)
        m_tf_data_received = yarp::os::Time::now();
        m_localization_data.x = pose[0];
        m_localization_data.y = pose[1];
        m_localization_data.theta = pose[5] * RAD2DEG;
    }
    if (current_time - m_tf_data_received > 0.1)
    {
        yWarning() << "No localization data received for more than 0.1s!";
    }
    
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

bool isaacLocalizerThread::initializeLocalization(yarp::dev::Map2DLocation& loc)
{
    m_localization_data.map_id = loc.map_id;
    
    if (m_iMap)
    {
        bool b = m_iMap->get_map(m_localization_data.map_id, m_current_map);
        if (b==false)
        {
            yError() << "Map "<<m_localization_data.map_id << " not found.";
        }
        else
        {
            publish_map();
        }
    }

    m_localization_data.x = loc.x;
    m_localization_data.y = loc.y;
    m_localization_data.theta = loc.theta;
    
    //send data to ISAAC localization module
    //@@@
    return true;
}

bool isaacLocalizerThread::getCurrentLoc(yarp::dev::Map2DLocation& loc)
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

    Bottle map_group = m_cfg.findGroup("MAP");
    if (map_group.isNull())
    {
        yError() << "Missing MAP group!";
        return false;
    }
    yDebug() << map_group.toString();

    //general group
    if (general_group.check("module_name") == false)
    {
        yError() << "Missing `module_name` in [GENERAL] group";
        return false;
    }
    m_module_name = general_group.find("module_name").asString();

    //initialize an occupancy grid publisher (every time the localization is re-initialized, the map is published too)
    //@@@

     //initialize an initial pose publisher
    //@@@

    //map server group
    yDebug() << map_group.toString();

    if (map_group.check("connect_to_yarp_mapserver") == false)
    {
        yError() << "Missing `connect_to_yarp_mapserver` in [MAP] group";
        return false;
    }
    m_use_map_server= (map_group.find("connect_to_yarp_mapserver").asInt()==1);

    //tf group
    if (tf_group.check("map_frame_id") == false)
    {
        yError() << "Missing `map_frame_id` in [TF] group";
        return false;
    }
    if (tf_group.check("robot_frame_id") == false)
    {
        yError() << "Missing `robot_frame_id` in [TF] group";
        return false;
    }
    m_frame_map_id = tf_group.find("map_frame_id").asString();
    m_frame_robot_id = tf_group.find("robot_frame_id").asString();


    //opens a client to receive localization data from transformServer
    Property options;
    options.put("device", "transformClient");
    options.put("local", "/"+m_module_name + "/TfClient");
    options.put("remote", "/transformServer");
    if (m_ptf.open(options) == false)
    {
        yError() << "Unable to open transform client";
        return false;
    }
    m_ptf.view(m_iTf);
    if (m_ptf.isValid() == false || m_iTf == 0)
    {
        yError() << "Unable to view iTransform interface";
        return false;
    }

    if (m_use_map_server)
    {
        //opens a client to send/received data from mapServer
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", "/" +m_module_name); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", "/mapServer");
        if (m_pmap.open(map_options) == false)
        {
            yWarning() << "Unable to open mapClient";
        }
        else
        {
            yInfo() << "Opened mapClient";
            m_pmap.view(m_iMap);
            if (m_pmap.isValid() == false || m_iMap == 0)
            {
                yError() << "Unable to view map interface";
                return false;
            }
        }
    }

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
    if (m_ptf.isValid())
    {
        m_ptf.close();
    }
    if (m_pmap.isValid())
    {
        m_pmap.close();
    }
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

    if (!thread->yarp::os::PeriodicThread::start())
    {
        delete thread;
        thread = NULL;
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
