/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "ros2Localizer.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/qos.hpp>

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

YARP_LOG_COMPONENT(ROS2_LOC, "navigation.ros2Localizer")


// Handler

void ros2LocalizerRPCHandler::setInterface(ros2Localizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool ros2LocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab32(Vocab32::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}

/////////////////////////////////////////////////////////////////////

// Localizer

bool ros2Localizer::open(yarp::os::Searchable& config)
{
    string cfg_temp = config.toString();
    Property p; p.fromString(cfg_temp);

    yCDebug(ROS2_LOC) << "ros2Localizer configuration: \n" << p.toString().c_str();

    Bottle general_group = p.findGroup("ROS2LOCALIZER_GENERAL");
    if (!general_group.isNull())
    {
        if (general_group.check("name")) {m_name = general_group.find("name").asString(); }
    }

    //create the m_thread
    m_thread = new ros2LocalizerThread(0.010, m_name, p);

    //initial location initialization
    Bottle initial_group = p.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing INITIAL_POS group!";
        return false;
    }
    if (initial_group.check("initial_x"))     { m_initial_loc.x = initial_group.find("initial_x").asFloat64(); }
    else { yCError(ROS2_LOC) << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y"))     { m_initial_loc.y = initial_group.find("initial_y").asFloat64(); }
    else { yCError(ROS2_LOC) << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asFloat64(); }
    else { yCError(ROS2_LOC) << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map"))   { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yCError(ROS2_LOC) << "missing initial_map param"; return false; }
    m_default_covariance_3x3.resize(3,3);
    m_default_covariance_3x3.zero();
    m_default_covariance_3x3[0][0] = 0.25;
    m_default_covariance_3x3[1][1] = 0.25;
    m_default_covariance_3x3[2][2] = 0.068538;

    //start the m_thread
    if (!m_thread->start())
    {
        delete m_thread;
        return false;
    }

    setInitialPose(m_initial_loc, m_default_covariance_3x3);

    bool ret = m_rpcPort.open(m_name+"/rpc");
    if (!ret)
    {
        yCError(ROS2_LOC) << "Unable to open module ports";
        return false;
    }

    m_rpcPortHandler.setInterface(this);
    m_rpcPort.setReader(m_rpcPortHandler);

    return true;
}

ros2Localizer::ros2Localizer()
{
    m_thread = NULL;
}

ros2Localizer::~ros2Localizer()
{
    if (m_thread)
    {
        delete m_thread;
        m_thread = NULL;
    }
}

bool ros2Localizer::close()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();
    return true;
}

ReturnValue ros2Localizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yCWarning(ROS2_LOC) << "Covariance matrix is not currently handled by ros2Localizer";
    m_thread->getCurrentLoc(loc);
    return ReturnValue_ok;
}

ReturnValue ros2Localizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
	if (m_thread)
	{
        yarp::sig::Matrix cov6x6(6,6);
	    cov6x6.zero();
	    cov6x6[0][0]=cov[0][0];
	    cov6x6[1][0]=cov[1][0];
	    cov6x6[5][0]=cov[2][0];

	    cov6x6[0][1]=cov[0][1];
	    cov6x6[1][1]=cov[1][1];
	    cov6x6[5][1]=cov[2][1];

	    cov6x6[0][5]=cov[0][2];
	    cov6x6[1][5]=cov[1][2];
	    cov6x6[5][5]=cov[2][2];
		if(!m_thread->initializeLocalization(loc, cov6x6))
        {
            yCError(ROS2_LOC) << "Unable to set initial pose";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
	}
	yCError(ROS2_LOC) << "ros2Localizer thread not running";
	return ReturnValue::return_code::return_value_error_generic;
}

ReturnValue  ros2Localizer::startLocalizationService()
{
    if (m_thread)
    {
        if(!m_thread->startLoc())
        {
            yCError(ROS2_LOC) << "Unable to start localization service";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
    }
    return ReturnValue::return_code::return_value_error_generic;
}

ReturnValue  ros2Localizer::stopLocalizationService()
{
    if (m_thread)
    {
        if(!m_thread->stopLoc())
        {
            yCError(ROS2_LOC) << "Unable to stop localization service";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
    }
    return ReturnValue::return_code::return_value_error_generic;
}

ReturnValue   ros2Localizer::getLocalizationStatus(LocalizationStatusEnum& status)
{
    status = LocalizationStatusEnum::localization_status_localized_ok;
    return ReturnValue_ok;
}

ReturnValue   ros2Localizer::getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses)
{
    if (m_thread)
    {
        if(!m_thread->getEstimatedPoses(poses))
        {
            yCError(ROS2_LOC) << "Unable to get estimated poses";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
    }
    yCError(ROS2_LOC) << "ros2Localizer thread not running";
    return ReturnValue::return_code::return_value_error_generic;
}

ReturnValue   ros2Localizer::getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc)
{
    if (m_thread)
    {
        if(!m_thread->getCurrentLoc(loc))
        {
            yCError(ROS2_LOC) << "Unable to get current position";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
    }
    yCError(ROS2_LOC) << "ros2Localizer thread not running";
    return ReturnValue::return_code::return_value_error_generic;
}

ReturnValue  ros2Localizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    odom = m_thread->getOdometry();
    return ReturnValue_ok;
}

ReturnValue   ros2Localizer::setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc)
{
    if (m_thread)
    {
        yarp::sig::Matrix cov6x6(6,6);
        cov6x6.zero();
        cov6x6[0][0]=m_default_covariance_3x3[0][0];
        cov6x6[1][0]=m_default_covariance_3x3[1][0];
        cov6x6[5][0]=m_default_covariance_3x3[2][0];

        cov6x6[0][1]=m_default_covariance_3x3[0][1];
        cov6x6[1][1]=m_default_covariance_3x3[1][1];
        cov6x6[5][1]=m_default_covariance_3x3[2][1];

        cov6x6[0][5]=m_default_covariance_3x3[0][2];
        cov6x6[1][5]=m_default_covariance_3x3[1][2];
        cov6x6[5][5]=m_default_covariance_3x3[2][2];
        if(!m_thread->initializeLocalization(loc, cov6x6))
        {
            yCError(ROS2_LOC) << "Unable to set initial pose";
            return ReturnValue::return_code::return_value_error_generic;
        }
        return ReturnValue_ok;
    }
    yCError(ROS2_LOC) << "ros2Localizer thread not running";
    return ReturnValue::return_code::return_value_error_generic;
}

///////////////////////////////////////

// Thread

ros2LocalizerThread::ros2LocalizerThread(double period, string _name, yarp::os::Searchable& _cfg) :
        PeriodicThread(period),
        m_name (_name), m_cfg(_cfg)
{
    m_iMap = 0;
    m_iTf = 0;
    m_tf_data_received = -1;
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = nan("");
    m_localization_data.y = nan("");
    m_localization_data.theta = nan("");

    m_seq_counter=0;
    m_rosTime = ros2TimeFromYarpNow();
}

bool ros2LocalizerThread::threadInit()
{
    // Check groups ///////////////////////////////////////////////////////////////////////////////////////////////////
    Bottle general_group = m_cfg.findGroup("ROS2LOCALIZER_GENERAL");
    if (general_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing ROS2LOCALIZER_GENERAL group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing LOCALIZATION group!";
        return false;
    }

    Bottle ros_group = m_cfg.findGroup("ROS2");
    if (ros_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing ROS2 group!";
        return false;
    }

    Bottle tf_group = m_cfg.findGroup("TF");
    if (tf_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing TF group!";
        return false;
    }

    Bottle map_group = m_cfg.findGroup("MAP");
    if (map_group.isNull())
    {
        yCError(ROS2_LOC) << "Missing MAP group!";
        return false;
    }

    // ROS2LOCALIZER_GENERAL group ////////////////////////////////////////////////////////////////////////////////////
    if (general_group.check("name"))
    {
        m_name = general_group.find("name").asString();
    }

    // LOCALIZATION group /////////////////////////////////////////////////////////////////////////////////////////////
    if (localization_group.check("localization_mode")==false)
    {
        yCError(ROS2_LOC) << "Missing localization_mode param!";
        return false;
    }
    string locmodestr = localization_group.find("localization_mode").asString();

    if (locmodestr == "tf")
    {
        m_loc_mode = use_tf_loc;
        yCInfo(ROS2_LOC) << "using localization from transform server";
    }
    else if (locmodestr == "ros")
    {
        m_loc_mode = use_ros_loc;
        yCInfo(ROS2_LOC) << "using localization from ros topic";
    }
    else
    {
        yCError(ROS2_LOC) << "invalid localization_mode param!";
        return false;
    }

    // ROS2 group /////////////////////////////////////////////////////////////////////////////////////////////////////
    //initialize an occupancy grid publisher (every time the localization is re-initialized, the map is published too)
    if(!m_node)
    {
        if(!ros_group.check("node_name"))
        {
            yCWarning(ROS2_LOC) << "No node_name specified. Using device name instead";
            m_nodeName = m_name;
        }
        else
        {
            m_nodeName = ros_group.find ("node_name").asString();
        }
        m_node = NodeCreator::createNode(m_nodeName);
    }

    //initialize an initial pose publisher
    if(!ros_group.check("initialpose_topic"))
    {
        yCError(ROS2_LOC) << "No initialpose_topic specified";
        return false;
    }
    m_topic_initial_pose = ros_group.find ("initialpose_topic").asString();
    m_ros2Publisher_initial_pose = m_node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(m_topic_initial_pose, 10);
    if (m_ros2Publisher_initial_pose == nullptr)
    {
        yCError(ROS2_LOC) << "localizationModule: unable to publish data on " << m_topic_initial_pose << " topic, check your yarp-ROS network configuration";
        return false;
    }

    if (!ros_group.check ("occupancygrid_topic"))
    {
        yCError(ROS2_LOC) << "No occupancygrid_topic specified";
        return false;
    }
    m_topic_occupancyGrid = ros_group.find ("occupancygrid_topic").asString();
    m_ros2Publisher_occupancyGrid = m_node->create_publisher<nav_msgs::msg::OccupancyGrid>(m_topic_occupancyGrid, rclcpp::QoS(10).reliable().durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));
    if (m_ros2Publisher_occupancyGrid == nullptr)
    {
        yCError(ROS2_LOC) << "localizationModule: unable to publish data on " << m_topic_occupancyGrid
                          << " topic, check your yarp-ROS network configuration";
        return false;
    }

    //initialize a subscriber for pose particles
    if (!ros_group.check("particles_topic"))
    {
        yCError(ROS2_LOC) << "No particles_topic specified";
        return false;
    }
    m_topic_particles = ros_group.find("particles_topic").asString();
    m_ros2Subscriber_particles = m_node->create_subscription<nav2_msgs::msg::ParticleCloud>(m_topic_particles, rclcpp::SensorDataQoS(), std::bind(&ros2LocalizerThread::particles_callback, this, _1));
    if (m_ros2Subscriber_particles == nullptr)
    {
        yCError(ROS2_LOC) << "localizationModule: unable to subscribe data on " << m_topic_particles << " topic, check your yarp-ROS network configuration";
        return false;
    }
    yCDebug(ROS2_LOC) << "opened " << m_topic_particles << " topic";

    if (m_loc_mode == use_ros_loc)
    {
        if (ros_group.check("currentpose_topic"))
        {
            m_topic_currpose = ros_group.find("currentpose_topic").asString();
            if (m_topic_currpose.empty())
            {
                yCError(ROS2_LOC) << "Invalid topic name for currentpose_topic";
                return false;
            }
            m_ros2Subscriber_current_pose = m_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(m_topic_currpose, rclcpp::SensorDataQoS(), std::bind(&ros2LocalizerThread::currPose_callback, this, _1));
            if (!m_ros2Subscriber_current_pose)
            {
                yCError(ROS2_LOC) << "localizationModule: unable to subscribe data on " << m_topic_currpose << " topic, check your yarp-ROS network configuration";
                return false;
            }
            yCDebug(ROS2_LOC) << "opened " << m_topic_currpose << " topic";
        }
        else
        {
            yCError(ROS2_LOC) << "Missing currentpose_topic param";
            return false;
        }
    }

    // MAP group //////////////////////////////////////////////////////////////////////////////////////////////////////
    yCDebug(ROS2_LOC) << map_group.toString();

    if (!map_group.check("connect_to_yarp_mapserver"))
    {
        yCError(ROS2_LOC) << "Missing `connect_to_yarp_mapserver` in [MAP] group";
        return false;
    }
    m_use_map_server= (map_group.find("connect_to_yarp_mapserver").asInt32()==1);

    // TF group ///////////////////////////////////////////////////////////////////////////////////////////////////////
    if (!tf_group.check("map_frame_id"))
    {
        yCError(ROS2_LOC) << "Missing `map_frame_id` in [TF] group";
        return false;
    }
    if (!tf_group.check("robot_frame_id"))
    {
        yCError(ROS2_LOC) << "Missing `robot_frame_id` in [TF] group";
        return false;
    }
    m_frame_map_id = tf_group.find("map_frame_id").asString();
    m_frame_robot_id = tf_group.find("robot_frame_id").asString();

    //opens a client to receive localization data from transformServer
    if (m_loc_mode == use_tf_loc)
    {
        Property options;
        options.put("device", "frameTransformClient");
        if(!tf_group.check("ft_client_config"))
        {
            yCWarning(ROS2_LOC) << "Parameter 'ft_client_config' missing in [TF] group. Using default value: 'ftc_yarp_only.xml'";
            options.put("filexml_option", "ftc_yarp_only.xml");
        }
        else
        {
            options.put("filexml_option", tf_group.find("ft_client_config").asString());
        }
        if(!tf_group.check("ft_client_prefix"))
        {
            yCWarning(ROS2_LOC) << "Parameter 'ft_client_prefix' missing in [TF] group. Using: '/" << m_name << "'";
            options.put("ft_client_prefix", m_name);
        }
        else
        {
            options.put("ft_client_prefix", tf_group.find("ft_client_prefix").asString());
        }

        if(!tf_group.check("ft_server_prefix"))
        {
            yCWarning(ROS2_LOC) << "Parameter 'ft_server_prefix' missing in [TF] group. Using an empty string";
        }
        else
        {
            options.put("ft_server_prefix", tf_group.find("ft_server_prefix").asString());
        }

        if (!m_ptf.open(options))
        {
            yCError(ROS2_LOC) << "Unable to open transform client";
            return false;
        }
        m_ptf.view(m_iTf);
        if (!m_ptf.isValid() || m_iTf == 0)
        {
            yCError(ROS2_LOC) << "Unable to view iTransform interface";
            return false;
        }
    }

    if (m_use_map_server)
    {
        //opens a client to send/received data from mapServer
        Property map_options;
        map_options.put("device", MAP_CLIENT_DEVICE_DEFAULT);
        map_options.put("local", m_name); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", MAP_REMOTE_PORT_DEFAULT);
        if (!m_pmap.open(map_options))
        {
            yCWarning(ROS2_LOC) << "Unable to open mapClient";
        }
        else
        {
            yCInfo(ROS2_LOC) << "Opened mapClient";
            m_pmap.view(m_iMap);
            if (!m_pmap.isValid() || m_iMap == 0)
            {
                yCError(ROS2_LOC) << "Unable to view map interface";
                return false;
            }
        }
    }

    return true;
}

void ros2LocalizerThread::publish_map()
{
    double tmp=0;
    nav_msgs::msg::OccupancyGrid ogrid;
    ogrid.info.height=m_current_map.height();
    ogrid.info.width=m_current_map.width();
    m_current_map.getResolution(tmp);
    ogrid.info.resolution=tmp;
    ogrid.header.frame_id="map";
    auto tempRos2Time = ros2TimeFromYarpNow();
    ogrid.header.stamp = tempRos2Time;
    ogrid.info.map_load_time.sec=0;
    ogrid.info.map_load_time.nanosec=0;
    double x, y, t;
    m_current_map.getOrigin(x,y,t);
    ogrid.info.origin.position.x=x;
    ogrid.info.origin.position.y=y;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0]=0; v[1]=0; v[2]=1; v[3]=t*DEG2RAD;
    q.fromAxisAngle(v);
    ogrid.info.origin.orientation.x = q.x();
    ogrid.info.origin.orientation.y = q.y();
    ogrid.info.origin.orientation.z = q.z();
    ogrid.info.origin.orientation.w = q.w();
    ogrid.data.resize(m_current_map.width()*m_current_map.height());
    int index=0;
    yarp::dev::Nav2D::XYCell cell;
    for (cell.y=m_current_map.height(); cell.y-- > 0;)
        for (cell.x=0; cell.x<m_current_map.width(); cell.x++)
        {
            m_current_map.getOccupancyData(cell,tmp);
            ogrid.data[index++]=(int)tmp;
        }

    m_ros2Publisher_occupancyGrid->publish(ogrid);
}

void ros2LocalizerThread::run()
{
    double current_time = yarp::os::Time::now();

    if (!m_spun)
    {
        m_innerSpinner = new Ros2Spinner(m_node);
        m_innerSpinner->start();
        m_spun = true;
    }

    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    lock_guard<std::mutex> lock(m_mutex);

    if(m_loc_mode == use_tf_loc)
    {
        yarp::sig::Vector iv;
        yarp::sig::Vector pose;
        iv.resize(6, 0.0);
        pose.resize(6, 0.0);
        bool r = m_iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);
        if (r) {
            //data is formatted as follows: x, y, angle (in degrees)
            m_tf_data_received = yarp::os::Time::now();
            m_localization_data.x = pose[0];
            m_localization_data.y = pose[1];
            m_localization_data.theta = pose[5] * RAD2DEG;

            //velocity estimation block
            if (1) { estimateOdometry(m_localization_data); }

        }
    }

    //FIXME
    //if (current_time - m_tf_data_received > 0.1)
    //
    //   yCWarning(ROS2_LOC) << "No localization data received for more than 0.1s!";
    //
}

bool ros2LocalizerThread::initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& roscov6x6)
{
    m_localization_data.map_id = loc.map_id;

    if (m_iMap)
    {
        bool b = m_iMap->get_map(m_localization_data.map_id, m_current_map);
        if (!b)
        {
            yCError(ROS2_LOC) << "Map "<<m_localization_data.map_id << " not found.";
        }
        else
        {
            publish_map();
        }
    }

    m_localization_data.x = loc.x;
    m_localization_data.y = loc.y;
    m_localization_data.theta = loc.theta;

    //send data to ROS localization module
    m_rosTime = ros2TimeFromYarpNow();
    geometry_msgs::msg::PoseWithCovarianceStamped pos;
    pos.header.frame_id="map";
    //pos.header.seq=m_seq_counter++;  // not present in ROS2 PoseWithCovarianceStamped header
    pos.header.stamp.sec=m_rosTime.sec;
    pos.header.stamp.nanosec=m_rosTime.nanosec;
    pos.pose.pose.position.x= loc.x;
    pos.pose.pose.position.y= loc.y;
    pos.pose.pose.position.z= 0;
    yarp::math::Quaternion q;
    yarp::sig::Vector v(4);
    v[0]=0;
    v[1]=0;
    v[2]=1;
    v[3]=loc.theta*DEG2RAD;
    q.fromAxisAngle(v);
    pos.pose.pose.orientation.x = q.x();
    pos.pose.pose.orientation.y = q.y();
    pos.pose.pose.orientation.z = q.z();
    pos.pose.pose.orientation.w = q.w();
    for (size_t ix=0; ix<6; ix++)
        for (size_t iy=0; iy<6; iy++)
            pos.pose.covariance[iy*6+ix] = roscov6x6[ix][iy];
    m_ros2Publisher_initial_pose->publish(pos);
    return true;
}

bool ros2LocalizerThread::getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc)
{
    loc = m_localization_data;
    return true;
}

bool ros2LocalizerThread::startLoc()
{
    this->resume();
    return true;
}
bool ros2LocalizerThread::stopLoc()
{
    this->suspend();
    return true;
}

bool ros2LocalizerThread::getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses)
{
    poses.clear();
    for (auto it = m_last_received_particles.particles.begin(); it!=m_last_received_particles.particles.end(); it++)
    {
        double xp = it->pose.position.x;
        double yp = it->pose.position.y;
        yarp::math::Quaternion q;
        q.x() = it->pose.orientation.x;
        q.y() = it->pose.orientation.y;
        q.z() = it->pose.orientation.z;
        q.w() = it->pose.orientation.w;
        yarp::sig::Vector v = q.toAxisAngle();
        double t = v[2];
        Map2DLocation loc(m_localization_data.map_id, xp, yp, t);
        poses.push_back(loc);
    }
    return true;
}

void ros2LocalizerThread::particles_callback(const nav2_msgs::msg::ParticleCloud poses)
{
    if (m_ros2Subscriber_particles)
    {
        if (!this->isRunning() ||
            this->isSuspended())
        {
            return;
        }

        m_last_received_particles = *(new nav2_msgs::msg::ParticleCloud(poses));
    }
}

void ros2LocalizerThread::currPose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped inputPose)
{
    if (m_ros2Subscriber_current_pose && m_loc_mode == use_ros_loc)
    {
        if (!this->isRunning() ||
            this->isSuspended())
        {
            return;
        }

        m_tf_data_received = yarp::os::Time::now();
        m_localization_data.x = inputPose.pose.pose.position.x;
        m_localization_data.y = inputPose.pose.pose.position.y;
        yarp::math::Quaternion q;
        q.x() = inputPose.pose.pose.orientation.x;
        q.y() = inputPose.pose.pose.orientation.y;
        q.z() = inputPose.pose.pose.orientation.z;
        q.w() = inputPose.pose.pose.orientation.w;
        yarp::sig::Vector v = q.toAxisAngle();
        double t = v[3]*v[2];
        m_localization_data.theta = t * RAD2DEG;

        //velocity estimation block
        if (1) { estimateOdometry(m_localization_data); }
    }
}

builtin_interfaces::msg::Time ros2LocalizerThread::ros2TimeFromYarpNow()
{
    double yarpTimeStamp = yarp::os::Time::now();
    builtin_interfaces::msg::Time toReturn;
    uint64_t sec_part;
    uint64_t nsec_part;
    sec_part = int(yarpTimeStamp); // (time / 1000000000UL);
    nsec_part = (yarpTimeStamp - sec_part)*1000000000UL;
    toReturn.sec = sec_part;
    toReturn.nanosec = (uint32_t)nsec_part;
    return toReturn;
}

void ros2LocalizerThread::threadRelease()
{
    if (m_ptf.isValid())
    {
        m_ptf.close();
    }
    if (m_pmap.isValid())
    {
        m_pmap.close();
    }

    if (m_ros2Publisher_occupancyGrid)
    {
        m_ros2Publisher_occupancyGrid = nullptr;
    }
    if (m_ros2Publisher_initial_pose)
    {
        m_ros2Publisher_initial_pose = nullptr;
    }
    if (m_ros2Subscriber_particles)
    {
        m_ros2Subscriber_particles = nullptr;
    }

    if (m_spun)
    {
        if (m_innerSpinner->isRunning())
        {
            m_innerSpinner->stop();
        }
        delete m_innerSpinner;
        m_innerSpinner = nullptr;
        m_spun = false;
    }
}

///////////////////////////////////////
