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
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>
#include <random>
#include <chrono>
#include <mutex>
#include "odomLocalizer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180


YARP_LOG_COMPONENT(ODOMLOC, "navigation.devices.odomLocalizer")
#define DISPLAY_PERIOD 1.0

void odomLocalizerRPCHandler::setInterface(odomLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool odomLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab32(Vocab32::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   odomLocalizer::getLocalizationStatus(LocalizationStatusEnum& status)
{
    status = LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   odomLocalizer::getEstimatedPoses(std::vector<Map2DLocation>& poses)
{
    poses.clear();
    Map2DLocation loc;
    m_thread->getCurrentLoc(loc);
    poses.push_back(loc);
#if 0
    //The following block is used only for development and debug purposes.
    //It should be never used in a real scenario
    for (int i = 0; i < 10; i++)
    {
        yarp::dev::Map2DLocation newloc=loc;
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::default_random_engine generator(seed);
        std::uniform_real_distribution<double> dist(-1, 1);
        std::uniform_real_distribution<double> dist_t(-180, 180);
        double numberx = dist(generator);
        double numbery = dist(generator);
        double numbert = dist_t(generator);
        newloc.x += numberx;
        newloc.y += numbery;
        newloc.theta += numbert;
        poses.push_back(newloc);
    }
#endif
    return true;
}

bool   odomLocalizer::getCurrentPosition(Map2DLocation& loc)
{
    m_thread->getCurrentLoc(loc);
    return true;
}

bool  odomLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
//    odom = m_thread->getOdometry(); //from the estimator
    m_thread->getLastOdometryDataFromPort(odom);
    return false;
}

bool   odomLocalizer::setInitialPose(const Map2DLocation& loc)
{
    m_thread->initializeLocalization(loc);
    return true;
}

//////////////////////////

odomLocalizerThread::odomLocalizerThread(double _period, string _name, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_name (_name), m_cfg(_cfg)
{
    m_last_port_odometryData_time = -1;
    m_last_statistics_printed = -1;

    m_current_loc.map_id = m_current_odom.map_id = m_initial_odom.map_id   = m_initial_loc.map_id = "unknown";
    m_current_loc.x      = m_current_odom.x      = m_initial_odom.x        = m_initial_loc.x = 0;
    m_current_loc.y      = m_current_odom.y      = m_initial_odom.y        = m_initial_loc.y = 0;
    m_current_loc.theta  = m_current_odom.theta  = m_initial_odom.theta    = m_initial_loc.theta = 0;
}

void odomLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    lock_guard<std::mutex> lock(m_mutex);
    yarp::dev::OdometryData *loc = m_port_odometry_input.read(false);
    if (loc)
    {
        m_last_port_odometryData = *loc;
        m_last_port_odometryData_time = yarp::os::Time::now();
        m_current_odom.x     = m_last_port_odometryData.odom_x;
        m_current_odom.y     = m_last_port_odometryData.odom_y;
        m_current_odom.theta = m_last_port_odometryData.odom_theta;

        double c = cos((-m_initial_odom.theta + m_initial_loc.theta)*DEG2RAD);
        double s = sin((-m_initial_odom.theta + m_initial_loc.theta)*DEG2RAD);
        double df_x = (m_current_odom.x - m_initial_odom.x);
        double df_y = (m_current_odom.y - m_initial_odom.y);
        m_current_loc.x = df_x * c + df_y * -s + m_initial_loc.x;
        m_current_loc.y = df_x * s + df_y * +c + m_initial_loc.y;

        m_current_loc.theta = m_current_odom.theta                   - m_initial_odom.theta + m_initial_loc.theta;

        if      (m_current_loc.theta >= +360) m_current_loc.theta -= 360;
        else if (m_current_loc.theta <= -360) m_current_loc.theta += 360;
    }
    if (current_time - m_last_port_odometryData_time > 0.1)
    {
        yCWarningThrottle(ODOMLOC,DISPLAY_PERIOD) << "No localization data received for more than 0.1s!";
    }
}

bool odomLocalizerThread::initializeLocalization(const Map2DLocation& loc)
{
    yCInfo(ODOMLOC) << "OdomLocalizer: Localization init request: (" << loc.map_id << ")";
    lock_guard<std::mutex> lock(m_mutex);
    m_initial_loc.map_id = loc.map_id;
    m_initial_loc.x = loc.x;
    m_initial_loc.y = loc.y;
    m_initial_loc.theta = loc.theta;
    m_initial_odom.x = m_current_odom.x;
    m_initial_odom.y = m_current_odom.y;
    m_initial_odom.theta = m_current_odom.theta;

    if (m_current_loc.map_id != m_initial_loc.map_id)
    {
        yCInfo(ODOMLOC) << "Map changed from: " << m_current_loc.map_id << " to: " << m_initial_loc.map_id;
        m_current_loc.map_id = m_initial_loc.map_id;
        //@@@TO BE COMPLETED
        m_current_loc.x = 0+m_initial_loc.x;
        m_current_loc.y = 0+m_initial_loc.y;
        m_current_loc.theta = 0+m_initial_loc.theta;
    }
    return true;
}

bool odomLocalizerThread::getCurrentLoc(Map2DLocation& loc)
{
    lock_guard<std::mutex> lock(m_mutex);
    loc = m_current_loc;
    return true;
}

bool odomLocalizerThread::threadInit()
{
    //configuration file checking
    Bottle general_group = m_cfg.findGroup("ODOMLOCALIZER_GENERAL");
    if (general_group.isNull())
    {
        yCError(ODOMLOC) << "Missing ODOMLOCALIZER_GENERAL group!";
        return false;
    }

    Bottle initial_group = m_cfg.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yCError(ODOMLOC) << "Missing INITIAL_POS group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yCError(ODOMLOC) << "Missing LOCALIZATION group!";
        return false;
    }

    Bottle tf_group = m_cfg.findGroup("TF");
    if (tf_group.isNull())
    {
        yCError(ODOMLOC) << "Missing TF group!";
        return false;
    }

    Bottle odometry_group = m_cfg.findGroup("ODOMETRY");
    if (odometry_group.isNull())
    {
        yCError(ODOMLOC) << "Missing ODOMETRY group!";
        return false;
    }

    //general group
    if (general_group.check("local_name")) { m_name = general_group.find("local_name").asString();}

    //odometry group
    if (odometry_group.check("odometry_broadcast_port") == false)
    {
        yCError(ODOMLOC) << "Missing `odometry_port` in [ODOMETRY] group";
        return false;
    }
    m_port_broadcast_odometry_name = odometry_group.find("odometry_broadcast_port").asString();

    //opens a YARP port to receive odometry data
    std::string odom_portname = m_name + "/odometry:i";
    bool b1 = m_port_odometry_input.open(odom_portname.c_str());
    bool b2 = yarp::os::Network::sync(odom_portname.c_str(), false);
    bool b3 = yarp::os::Network::connect(m_port_broadcast_odometry_name.c_str(), odom_portname.c_str());
    if (b1 == false || b2 == false || b3 == false)
    {
        yCError(ODOMLOC) << "Unable to initialize odometry port connection from " << m_port_broadcast_odometry_name.c_str() << "to:" << odom_portname.c_str();
        return false;
    }

    //initial location initialization
    Map2DLocation tmp_loc;
    if (initial_group.check("initial_x")) { tmp_loc.x = initial_group.find("initial_x").asFloat64(); }
    else { yCError(ODOMLOC) << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y")) { tmp_loc.y = initial_group.find("initial_y").asFloat64(); }
    else { yCError(ODOMLOC) << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { tmp_loc.theta = initial_group.find("initial_theta").asFloat64(); }
    else { yCError(ODOMLOC) << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map")) { tmp_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yCError(ODOMLOC) << "missing initial_map param"; return false; }
    this->initializeLocalization(tmp_loc);

   return true;
}

void odomLocalizerThread::threadRelease()
{

}

bool odomLocalizerThread::getLastOdometryDataFromPort(yarp::dev::OdometryData& odom)
{
    lock_guard<std::mutex> lock(m_mutex);
    odom = this->m_last_port_odometryData;
    return true;
}

bool odomLocalizer::open(yarp::os::Searchable& config)
{
    string cfg_temp = config.toString();
    Property p; p.fromString(cfg_temp);

    yCDebug(ODOMLOC) << "odomLocalizer configuration: \n" << p.toString().c_str();

    Bottle general_group = p.findGroup("ODOMLOCALIZER_GENERAL");
    if (general_group.isNull()==false)
    {
        if (general_group.check("local_name")) { m_name = general_group.find("local_name").asString(); }
    }
    bool ret = m_rpcPort.open(m_name+"/rpc");
    if (ret == false)
    {
        yCError(ODOMLOC) << "Unable to open module ports";
        return false;
    }

    m_thread = new odomLocalizerThread(0.010, m_name, p);

    if (!m_thread->start())
    {
        delete m_thread;
        m_thread = NULL;
        return false;
    }

    m_rpcPortHandler.setInterface(this);
    m_rpcPort.setReader(m_rpcPortHandler);

    return true;
}

odomLocalizer::odomLocalizer()
{
    m_thread = NULL;
}

odomLocalizer::~odomLocalizer()
{
    if (m_thread)
    {
        delete m_thread;
        m_thread = NULL;
    }
}

bool odomLocalizer::close()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();
    return true;
}
 
bool   odomLocalizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yCWarning(ODOMLOC) << "Covariance matrix is not currently handled by odomLocalizer";
    m_thread->getCurrentLoc(loc);
    return true;
}

bool   odomLocalizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yCWarning(ODOMLOC) << "Covariance matrix is not currently handled by odomLocalizer";
    m_thread->initializeLocalization(loc);
    return true;
}

bool    odomLocalizer::startLocalizationService()
{
    yCError(ODOMLOC) << "Not yet implemented";
    return false;
}

bool    odomLocalizer::stopLocalizationService()
{
    yCError(ODOMLOC) << "Not yet implemented";
    return false;
}
