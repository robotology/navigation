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

YARP_LOG_COMPONENT(GAZEBO_LOC, "navigation.gazeboLocalizer")

void gazeboLocalizerRPCHandler::setInterface(gazeboLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool gazeboLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab32(Vocab32::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


ReturnValue   gazeboLocalizer::getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status)
{
    status = yarp::dev::Nav2D::LocalizationStatusEnum::localization_status_localized_ok;
    return ReturnValue_ok;
}

ReturnValue   gazeboLocalizer::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    poses.clear();
    Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return ReturnValue_ok;
}

ReturnValue   gazeboLocalizer::getCurrentPosition(Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return ReturnValue_ok;
}

ReturnValue  gazeboLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    odom = thread->getOdometry();
    return ReturnValue_ok;
}

ReturnValue   gazeboLocalizer::setInitialPose(const Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return ReturnValue_ok;
}

//////////////////////////

gazeboLocalizerThread::gazeboLocalizerThread(double _period, string _name, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_name(_name), m_cfg(_cfg)
{
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = 0;
    m_localization_data.y = 0;
    m_localization_data.theta = 0;
}

void gazeboLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
        yCDebug(GAZEBO_LOC) << "gazeboLocalizerThread running with period: " << this->getPeriod();
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

    estimateOdometry(m_localization_data);
}

bool gazeboLocalizerThread::initializeLocalization(const Map2DLocation& loc)
{
    yCInfo(GAZEBO_LOC) << "gazeboLocalizerThread: Localization init request: (" << loc.map_id << ")";
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
        yCError(GAZEBO_LOC) << "open_gazebo() failed, unable to connect port " << m_local_gazebo_port_name << " with " << m_remote_gazebo_port_name;
        return false;
    }
    yCError(GAZEBO_LOC) << "open_gazebo() failed, unable to open port " << m_local_gazebo_port_name;
    return false;
}

bool gazeboLocalizerThread::threadInit()
{
    //configuration file checking
    Bottle general_group = m_cfg.findGroup("GAZEBOLOCALIZER_GENERAL");
    if (general_group.isNull())
    {
        yCError(GAZEBO_LOC) << "Missing GAZEBOLOCALIZER_GENERAL group!";
        return false;
    }

    Bottle initial_group = m_cfg.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yCError(GAZEBO_LOC) << "Missing INITIAL_POS group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yCError(GAZEBO_LOC) << "Missing LOCALIZATION group!";
        return false;
    }

    m_local_gazebo_port_name = m_name + "/gazebo_rpc";

    if (general_group.check("robot_name")) { m_object_name = general_group.find("robot_name").asString(); }
    else { yCError(GAZEBO_LOC) << "missing robot_name param"; yCError(GAZEBO_LOC) << "I need the name of the object to be localized!"; return false; }

    if (general_group.check("world_interface_port")) { m_remote_gazebo_port_name = general_group.find("world_interface_port").asString(); }
    else { yCError(GAZEBO_LOC) << "missing world_interface_port param"; return false; }

    //starts communication with gazebo
    if (!open_gazebo())
    {
        yCError(GAZEBO_LOC) << "Unable to start communication with gazebo!";
        return false;
    }

    //initial location initialization
    Map2DLocation tmp_loc;
    if (initial_group.check("map_transform_x")) { tmp_loc.x = initial_group.find("map_transform_x").asFloat64(); }
    else { yCError(GAZEBO_LOC) << "missing map_transform_x param"; return false; }
    if (initial_group.check("map_transform_y")) { tmp_loc.y = initial_group.find("map_transform_y").asFloat64(); }
    else { yCError(GAZEBO_LOC) << "missing map_transform_y param"; return false; }
    if (initial_group.check("map_transform_t")) { tmp_loc.theta = initial_group.find("map_transform_t").asFloat64(); }
    else { yCError(GAZEBO_LOC) << "missing map_transform_t param"; return false; }
    if (initial_group.check("initial_map")) { tmp_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yCError(GAZEBO_LOC) << "missing initial_map param"; return false; }
    this->initializeLocalization(tmp_loc);

   return true;
}

void gazeboLocalizerThread::threadRelease()
{
    yCDebug(GAZEBO_LOC) << "Closing ports";
    m_port_gazebo_comm.interrupt();
    m_port_gazebo_comm.close();
}

bool gazeboLocalizer::open(yarp::os::Searchable& config)
{
    string cfg_temp = config.toString();
    Property p; p.fromString(cfg_temp);

    yCDebug(GAZEBO_LOC) << "gazeboLocalizer configuration: \n" << p.toString().c_str();

    Bottle general_group = p.findGroup("GAZEBOLOCALIZER_GENERAL");
    double period = 0.010;
    if (general_group.isNull() == false)
    {
        if (general_group.check("name")) { m_name = general_group.find("local_name").asString(); }
        if (general_group.check("period")) { period = general_group.find("period").asFloat64(); }
    }

    thread = new gazeboLocalizerThread(period, m_name, p);

    if (!thread->start())
    {
        delete thread;
        thread = NULL;
        return false;
    }

    bool ret = rpcPort.open(m_name+"/rpc");
    if (ret == false)
    {
        yCError(GAZEBO_LOC) << "Unable to open module ports";
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
    yCDebug(GAZEBO_LOC) << "gazeboLocalizer terminated";
    return true;
}
 
ReturnValue   gazeboLocalizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yCWarning(GAZEBO_LOC) << "Covariance matrix is not currently handled by gazeboLocalizer";
    thread->getCurrentLoc(loc);
    return ReturnValue_ok;
}

ReturnValue   gazeboLocalizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yCWarning(GAZEBO_LOC) << "Covariance matrix is not currently handled by gazeboLocalizer";
    thread->initializeLocalization(loc);
    return ReturnValue_ok;
}

ReturnValue    gazeboLocalizer::startLocalizationService()
{
    yCError(GAZEBO_LOC) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}

ReturnValue    gazeboLocalizer::stopLocalizationService()
{
    yCError(GAZEBO_LOC) << "Not yet implemented";
    return ReturnValue::return_code::return_value_error_not_implemented_by_device;
}
