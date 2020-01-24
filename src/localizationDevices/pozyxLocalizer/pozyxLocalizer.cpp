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
#include <mutex>
#include <math.h>
#include "pozyxLocalizer.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

#define SIMULATE_POZYX

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


bool   pozyxLocalizer::getLocalizationStatus(LocalizationStatusEnum& status)
{
    status = LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   pozyxLocalizer::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    poses.clear();
    Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return true;
}

bool   pozyxLocalizer::getCurrentPosition(Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool  pozyxLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    yError() << " pozyxLocalizer::getEstimatedOdometry is not yet implemented";
    return false;
}

bool   pozyxLocalizer::setInitialPose(const Map2DLocation& loc)
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

    m_iMap = 0;
    m_publish_anchors_as_map_locations = false;
    m_remote_map = "/mapServer";
    m_local_name = "/pozyxLocalizer";
}

void pozyxLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    lock_guard<std::mutex> lock(m_mutex);

    //@@@@READ DATA FROM DEVICE here
    m_pozyx_data.x=0;
    m_pozyx_data.y=0;
    m_pozyx_data.theta=0;

    //@@@@COMPUTE LOCALIZATION DATA here
    double angle = m_map_to_pozyx_transform.theta * DEG2RAD;
    m_localization_data.x     = m_map_to_pozyx_transform.x + cos (angle) * m_pozyx_data.x - sin (angle) * m_pozyx_data.y;
    m_localization_data.y     = m_map_to_pozyx_transform.y + sin (angle) * m_pozyx_data.x + cos (angle) * m_pozyx_data.y;
    m_localization_data.theta = m_pozyx_data.theta + m_map_to_pozyx_transform.theta;
    if      (m_localization_data.theta >= +360) m_localization_data.theta -= 360;
    else if (m_localization_data.theta <= -360) m_localization_data.theta += 360;
}

bool pozyxLocalizerThread::initializeLocalization(const Map2DLocation& loc)
{
    yInfo() << "pozyxLocalizerThread: Localization init request: (" << loc.map_id << ")";
    lock_guard<std::mutex> lock(m_mutex);
    //@@@@ put some check here on validity of loc
    m_localization_data.map_id = loc.map_id;
    m_map_to_pozyx_transform.map_id = loc.map_id;
    m_map_to_pozyx_transform.x = loc.x;
    m_map_to_pozyx_transform.y = loc.y;
    m_map_to_pozyx_transform.theta = loc.theta;

    if (get_anchors_location())
    {
        double angle = m_map_to_pozyx_transform.theta * DEG2RAD;
        for (size_t i = 0; i < m_anchors_pos.size(); i++)
        {
            Map2DLocation tmp = m_anchors_pos[i];
            m_anchors_pos[i].x = m_map_to_pozyx_transform.x + cos(angle) * tmp.x - sin(angle) * tmp.y;
            m_anchors_pos[i].y = m_map_to_pozyx_transform.y + sin(angle) * tmp.x + cos(angle) * tmp.y;
            m_anchors_pos[i].theta = 0;
        }

        if (m_publish_anchors_as_map_locations)
        {
            if (m_iMap)
            {
                publish_anchors_location();
            }
        }
    }
    else
    {
        yWarning() << "No anchors found";
    }

    return true;
}

bool pozyxLocalizerThread::getCurrentLoc(Map2DLocation& loc)
{
    lock_guard<std::mutex> lock(m_mutex);
    loc = m_localization_data;
    return true;
}


bool pozyxLocalizerThread::get_anchors_location()
{
    m_anchors_pos.clear();
#ifdef SIMULATE_POZYX
    m_anchors_pos.push_back(Map2DLocation("test_map", -1, +1, 0 ));
    m_anchors_pos.push_back(Map2DLocation("test_map", -1, -1, 0 ));
    m_anchors_pos.push_back(Map2DLocation("test_map",  0,  0, 0 ));
    m_anchors_pos.push_back(Map2DLocation("test_map", -1,  0, 0 ));
#else
   for (size_t i = 0; i < 0; i++)
   {
       Map2DLocation loc;
       //@@@@READ DATA FROM DEVICE here and fill loc
       m_anchors_pos.push_back(loc);
   }
#endif
   return true;
}

bool pozyxLocalizerThread::open_pozyx()
{
#ifdef SIMULATE_POZYX
    return true;
#else
    //@@@@OPEN DEVICE here
    return true;
#endif
}

bool pozyxLocalizerThread::publish_anchors_location()
{
    if (!m_iMap)
    {
        yError() << "pozyxLocalizerThread::publish_anchors_location() failed";
        return false;
    }

    if (m_anchors_pos.size() == 0)
    {
        yError() << "pozyxLocalizerThread::publish_anchors_location() no anchors available";
        return false;
    }

    for (size_t i = 0; i < m_anchors_pos.size(); i++)
    {
        string anchor_name = "anchor_" + std::to_string(i);
        m_iMap->deleteLocation(anchor_name);
        bool ret = m_iMap->storeLocation(anchor_name, m_anchors_pos[i]);
        if (!ret)
        {
            yError() << "Failed to store position of: " << anchor_name;
        }
    }

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
    if (!open_pozyx())
    {
        yError() << "Unable to open pozyx device";
        return false;
    }
    if (!get_anchors_location())
    {
        yError() << "Unable to open map2DClient";
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

    if (general_group.check("publish_anchors")) 
    {   m_publish_anchors_as_map_locations = general_group.find("publish_anchors").asBool();  }
    else { yError() << "missing publish_anchors param"; return false; }

    if (general_group.check("local_name"))
    {
        m_local_name_prefix = general_group.find("local_name").asString();
    }
    else
    {
        yInfo() << "local_name parameter not set. Using:" << m_local_name_prefix;
    }

    if (general_group.check("remote_mapServer"))
    {
        m_remote_map = general_group.find("remote_mapServer").asString();
    }
    else
    {
        yInfo() << "remote_mapServer parameter not set. Using:" << m_remote_map;
    }

    //the optional map client
    if (m_publish_anchors_as_map_locations)
    {
        //open the map interface
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", m_local_name_prefix + "/map2DClient");
        map_options.put("remote", m_remote_map);
        if (m_pMap.open(map_options) == false)
        {
            yError() << "Unable to open map2DClient";
            return false;
        }
        m_pMap.view(m_iMap);
        if (m_iMap == 0)
        {
            yError() << "Unable to open map interface";
            return false;
        }
        //publish the anchor location
        publish_anchors_location();
    }

   return true;
}

void pozyxLocalizerThread::threadRelease()
{

}


bool pozyxLocalizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "pozyxLocalizer";
    std::string file_name = "pozyxLocalizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "pozyxLocalizer configuration: \n" << p.toString().c_str();

    thread = new pozyxLocalizerThread(0.010, p);

    if (!thread->start())
    {
        delete thread;
        thread = NULL;
        return false;
    }

    std::string local_name = "pozyxLocalizer";
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
 

bool pozyxLocalizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by pozyxLocalizer";
    thread->getCurrentLoc(loc);
    return true;
}

bool pozyxLocalizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by pozyxLocalizer";
    thread->initializeLocalization(loc);
    return true;
}

bool  pozyxLocalizer::startLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}

bool  pozyxLocalizer::stopLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}
