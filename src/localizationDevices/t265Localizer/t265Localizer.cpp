/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Quaternion.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <mutex>
#include <math.h>
#include "t265Localizer.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

//#define SIMULATE_T265

void t265LocalizerRPCHandler::setInterface(t265Localizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool t265LocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab(Vocab::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   t265Localizer::getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status)
{
    status = yarp::dev::LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   t265Localizer::getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses)
{
    poses.clear();
    yarp::dev::Nav2D::Map2DLocation loc;
    thread->getCurrentLoc(loc);
    poses.push_back(loc);
    return true;
}

bool   t265Localizer::getCurrentPosition(Map2DLocation& loc)
{
    thread->getCurrentLoc(loc);
    return true;
}

bool   t265Localizer::setInitialPose(const Map2DLocation& loc)
{
    thread->initializeLocalization(loc);
    return true;
}

bool   t265Localizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by t265Localizer";
    thread->getCurrentLoc(loc);
    return true;
}

bool   t265Localizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
    yWarning() << "Covariance matrix is not currently handled by t265Localizer";
    thread->initializeLocalization(loc);
    return true;
}

bool    t265Localizer::startLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}

bool    t265Localizer::stopLocalizationService()
{
    yError() << "Not yet implemented";
    return false;
}

//////////////////////////
odometry_handler::odometry_handler(const rs2::device& dev) : m_rs_odometry_handler(rs2::wheel_odometer(dev.first<rs2::wheel_odometer>()))
{
   m_linear_velocity.x = 0;
   m_linear_velocity.y = 0;
   m_linear_velocity.z = 0;
   m_counter = 0;
}

void odometry_handler::onRead(yarp::dev::OdometryData& b)
{
    //the following lines perform a reference frame tranformation
    m_linear_velocity.x = 0;
    m_linear_velocity.y = 0;
    m_linear_velocity.z = b.odom_x;
//    m_rs_odometry_handler.send_wheel_odometry(0, 0, m_linear_velocity);
    m_rs_odometry_handler.send_wheel_odometry(0, m_counter, m_linear_velocity);
    m_counter++;
}

//////////////////////////
t265LocalizerThread::t265LocalizerThread(double _period, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_cfg(_cfg)
{
    m_odometry_handler = nullptr;
    m_last_statistics_printed = -1;

    m_iMap = 0;
    m_remote_map = "/mapServer";
    m_local_name = "/t265Localizer";

    m_current_loc.map_id = m_current_device_data.map_id = m_initial_device_data.map_id = m_initial_loc.map_id = "unknown";
    m_current_loc.x = m_current_device_data.x = m_initial_device_data.x = m_initial_loc.x = 0;
    m_current_loc.y = m_current_device_data.y = m_initial_device_data.y = m_initial_loc.y = 0;
    m_current_loc.theta = m_current_device_data.theta = m_initial_device_data.theta = m_initial_loc.theta = 0;
}

void t265LocalizerThread::odometry_update()
{

}

void t265LocalizerThread::run()
{
   double current_time = yarp::os::Time::now();

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    lock_guard<std::mutex> lock(m_mutex);

    //read data from the device
    // Wait for the next set of frames from the camera
    auto frames = m_realsense_pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    rs2_pose pose_data = f.as<rs2::pose_frame>().get_pose_data();
    if (0)
    {
        m_current_device_data.x = pose_data.translation.x;
        m_current_device_data.y = pose_data.translation.y;
        yarp::math::Quaternion q(pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w);
        auto m = q.toRotationMatrix3x3();
        auto v = yarp::math::dcm2euler(m);
        m_current_device_data.theta = v[2] * RAD2DEG;
    }
    else
    {
        m_current_device_data.x = pose_data.translation.z;
        m_current_device_data.y = pose_data.translation.x;
        yarp::math::Quaternion q (pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w);
        auto m = q.toRotationMatrix3x3();
        auto v = yarp::math::dcm2euler(m);
        m_current_device_data.theta =  v[1]*RAD2DEG;
    }
    yDebug() << "device pose (x y t) before relocation:" << m_current_device_data.x << m_current_device_data.y << m_current_device_data.theta;

    //relocate data in robot frame
    relocate_data(m_current_device_data);
    yDebug() << "device pose (x y t) after relocation: " << m_current_device_data.x << m_current_device_data.y << m_current_device_data.theta;

    //compute data localization
    double c = cos((-m_initial_device_data.theta + m_initial_loc.theta)*DEG2RAD);
    double s = sin((-m_initial_device_data.theta + m_initial_loc.theta)*DEG2RAD);
    double df_x = (m_current_device_data.x - m_initial_device_data.x);
    double df_y = (m_current_device_data.y - m_initial_device_data.y);
    m_current_loc.x = df_x * c + df_y * -s + m_initial_loc.x;
    m_current_loc.y = df_x * s + df_y * +c + m_initial_loc.y;

    m_current_loc.theta = m_current_device_data.theta - m_initial_device_data.theta + m_initial_loc.theta;

    if (m_current_loc.theta >= +360) m_current_loc.theta -= 360;
    else if (m_current_loc.theta <= -360) m_current_loc.theta += 360;
}

bool t265LocalizerThread::initializeLocalization(const Map2DLocation& loc)
{
    yInfo() << "t265LocalizerThread: Localization init request: (" << loc.map_id << ")";
    lock_guard<std::mutex> lock(m_mutex);
    m_initial_loc.map_id = loc.map_id;
    m_initial_loc.x = loc.x;
    m_initial_loc.y = loc.y;
    m_initial_loc.theta = loc.theta;
    m_initial_device_data.x = m_current_device_data.x;
    m_initial_device_data.y = m_current_device_data.y;
    m_initial_device_data.theta = m_current_device_data.theta;

    if (m_current_loc.map_id != m_initial_loc.map_id)
    {
        yInfo() << "Map changed from: " << m_current_loc.map_id << " to: " << m_initial_loc.map_id;
        m_current_loc.map_id = m_initial_loc.map_id;
        //@@@TO BE COMPLETED
        m_current_loc.x = 0 + m_initial_loc.x;
        m_current_loc.y = 0 + m_initial_loc.y;
        m_current_loc.theta = 0 + m_initial_loc.theta;
    }
    return true;
}

bool t265LocalizerThread::getCurrentLoc(Map2DLocation& loc)
{
    lock_guard<std::mutex> lock(m_mutex);
    loc = m_current_loc;
    return true;
}

bool t265LocalizerThread::open_device()
{
#ifdef SIMULATE_T265
    return true;
#else
    //initialize Realsense device
    try
    {
        m_realsense_cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
        m_realsense_pipe.start(m_realsense_cfg);
    }
    catch (const rs2::error & e)
    {
        yError() << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what();
        return false;
    }
    return true;
#endif
}

bool t265LocalizerThread::threadInit()
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
    if (general_group.check("local_name")) { m_local_name = general_group.find("local_name").asString(); }

    //initialize the device relocation system on the robot
    movable_localization_device::init(m_cfg);

    //opens the device communication
    if (!open_device())
    {
        yError() << "Unable to open t265 device";
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
    if (0)
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
    }

    //the odometry port
    if (m_odometry_handler)
    {
        m_odometry_handler = new odometry_handler(m_realsense_pipe.get_active_profile().get_device());
        m_odometry_handler->useCallback();  // input should go to onRead() callback
        string odometry_port_name = m_local_name + "/odometry:i";
        if (m_odometry_handler->open(odometry_port_name) == false)
        {
            yError() << "Unable to open odometry port" << odometry_port_name;
            return false;
        }
    }
    else
    {
       yError() << "m_odometry_handler not initialized"; 
       return false;
    }

    return true;
}

void t265LocalizerThread::threadRelease()
{
   if (m_odometry_handler)
   {
       m_odometry_handler->interrupt();
       m_odometry_handler->close();
   }
}


bool t265Localizer::open(yarp::os::Searchable& config)
{
    yDebug() << "config configuration: \n" << config.toString().c_str();

    std::string context_name = "t265Localizer";
    std::string file_name = "t265Localizer.ini";

    if (config.check("context"))   context_name = config.find("context").asString();
    if (config.check("from")) file_name = config.find("from").asString();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext(context_name.c_str());
    rf.setDefaultConfigFile(file_name.c_str());

    Property p;
    std::string configFile = rf.findFile("from");
    if (configFile != "") p.fromConfigFile(configFile.c_str());
    yDebug() << "t265Localizer configuration: \n" << p.toString().c_str();

    thread = new t265LocalizerThread(0.010, p);

    if (!thread->start())
    {
        delete thread;
        thread = NULL;
        return false;
    }

    std::string local_name = "t265Localizer";
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

t265Localizer::t265Localizer()
{
    thread = NULL;
}

t265Localizer::~t265Localizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool t265Localizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}
 
