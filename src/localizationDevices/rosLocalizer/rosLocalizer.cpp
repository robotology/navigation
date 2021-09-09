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
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/rosmsg/geometry_msgs/PoseStamped.h>
#include <yarp/rosmsg/geometry_msgs/PoseWithCovarianceStamped.h>
#include <yarp/rosmsg/nav_msgs/OccupancyGrid.h>
#include <math.h>
#include <mutex>
#include "rosLocalizer.h"

using namespace std;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

YARP_LOG_COMPONENT(ROS_LOC, "navigation.rosLocalizer")

void rosLocalizerRPCHandler::setInterface(rosLocalizer* iface)
{
    this->interface = iface;
}

//This function parses the user commands received through the RPC port
bool rosLocalizerRPCHandler::respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
{
    reply.clear();
    reply.addVocab32(Vocab32::encode("many"));
    reply.addString("Not yet Implemented");
    return true;
}


bool   rosLocalizer::getLocalizationStatus(LocalizationStatusEnum& status)
{
    status = LocalizationStatusEnum::localization_status_localized_ok;
    return true;
}

bool   rosLocalizer::getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses)
{
    if (thread)
    {
        return thread->getEstimatedPoses(poses);
    }
    yCError(ROS_LOC) << "rosLocalizer thread not running";
    return false;
}

bool   rosLocalizer::getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc)
{
    if (thread)
    {
        return thread->getCurrentLoc(loc);
    }
    yCError(ROS_LOC) << "rosLocalizer thread not running";
    return false;
}

bool  rosLocalizer::getEstimatedOdometry(yarp::dev::OdometryData& odom)
{
    odom = thread->getOdometry();
    return true;
}

bool   rosLocalizer::setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc)
{
    if (thread)
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
        return thread->initializeLocalization(loc, cov6x6);
    }
    yCError(ROS_LOC) << "rosLocalizer thread not running";
    return false;
}

//////////////////////////

rosLocalizerThread::rosLocalizerThread(double _period, string _name, yarp::os::Searchable& _cfg) : PeriodicThread(_period), m_name (_name), m_cfg(_cfg)
{
    m_iMap = 0;
    m_iTf = 0;
    m_rosNode = 0;
    m_ros_enabled = false;
    m_tf_data_received = -1;
    m_last_statistics_printed = -1;

    m_localization_data.map_id = "unknown";
    m_localization_data.x = nan("");
    m_localization_data.y = nan("");
    m_localization_data.theta = nan("");
      
    m_seq_counter=0;
    m_rosTime = yarp::os::Time::now();
}


void rosLocalizerThread::publish_map()
{
    double tmp=0;
    yarp::rosmsg::nav_msgs::OccupancyGrid& ogrid = m_rosPublisher_occupancyGrid.prepare();
    ogrid.clear();
    ogrid.info.height=m_current_map.height();
    ogrid.info.width=m_current_map.width();
    m_current_map.getResolution(tmp);
    ogrid.info.resolution=tmp;
    ogrid.header.frame_id="map";
    ogrid.info.map_load_time.sec=0;
    ogrid.info.map_load_time.nsec=0;
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
      
    m_rosPublisher_occupancyGrid.write();
}

void rosLocalizerThread::run()
{
    double current_time = yarp::os::Time::now();
    
    //print some stats every 10 seconds
    if (current_time - m_last_statistics_printed > 10.0)
    {
        m_last_statistics_printed = yarp::os::Time::now();
    }

    lock_guard<std::mutex> lock(m_mutex);
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

        //velocity estimation block
        if (1) { estimateOdometry(m_localization_data); }

    }
    if (current_time - m_tf_data_received > 0.1)
    {
        yCWarning(ROS_LOC) << "No localization data received for more than 0.1s!";
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

bool rosLocalizerThread::initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& roscov6x6)
{
    m_localization_data.map_id = loc.map_id;
    
    if (m_iMap)
    {
        bool b = m_iMap->get_map(m_localization_data.map_id, m_current_map);
        if (b==false)
        {
            yCError(ROS_LOC) << "Map "<<m_localization_data.map_id << " not found.";
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
    m_rosTime = (yarp::os::Time::now());
    yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped& pos = m_rosPublisher_initial_pose.prepare();
    pos.clear();
    pos.header.frame_id="map";
    pos.header.seq=m_seq_counter++;
    pos.header.stamp.sec=m_rosTime.sec;
    pos.header.stamp.nsec=m_rosTime.nsec;
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
    m_rosPublisher_initial_pose.write();
    return true;
}

bool rosLocalizerThread::getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc)
{
    loc = m_localization_data;
    return true;
}

bool rosLocalizerThread::startLoc()
{
    this->resume();
    return true;
}
bool rosLocalizerThread::stopLoc()
{
    this->suspend();
    return true;
}


bool rosLocalizerThread::getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses)
{
	if (m_rosSubscriber_particles.asPort().isOpen())
    {
        if (this->isRunning()==false ||
            this->isSuspended() == true)
        {
			poses.clear();
			return true;
	    }
    
        yarp::rosmsg::geometry_msgs::PoseArray* poseArr = m_rosSubscriber_particles.read(false);
		if (poseArr)
		{
			m_last_received_particles = *poseArr;
	    }
	    
    	poses.clear();
		for (auto it = m_last_received_particles.poses.begin(); it!=m_last_received_particles.poses.end(); it++)
		{
			double xp = it->position.x;
			double yp = it->position.y;
			yarp::math::Quaternion q;
			q.x() = it->orientation.x;
			q.y() = it->orientation.y;
			q.z() = it->orientation.z;
			q.w() = it->orientation.w;
			yarp::sig::Vector v = q.toAxisAngle();
			double t = v[2];
			Map2DLocation loc(m_localization_data.map_id, xp, yp, t);
			poses.push_back(loc);
		}
        return true;
    }
    //data not available. Returning true to prevent message flooding.
    poses.clear();
    return true; 
}

bool rosLocalizerThread::threadInit()
{ 
    //configuration file checking
    Bottle general_group = m_cfg.findGroup("ROSLOCALIZER_GENERAL");
    if (general_group.isNull())
    {
        yCError(ROS_LOC) << "Missing ROSLOCALIZER_GENERAL group!";
        return false;
    }

    Bottle localization_group = m_cfg.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yCError(ROS_LOC) << "Missing LOCALIZATION group!";
        return false;
    }

    Bottle ros_group = m_cfg.findGroup("ROS");

    Bottle tf_group = m_cfg.findGroup("TF");
    if (tf_group.isNull())
    {
        yCError(ROS_LOC) << "Missing TF group!";
        return false;
    }

    Bottle map_group = m_cfg.findGroup("MAP");
    if (map_group.isNull())
    {
        yCError(ROS_LOC) << "Missing MAP group!";
        return false;
    }
    yCDebug(ROS_LOC) << map_group.toString();

    m_rosNode = new yarp::os::Node(m_name);
    
    //initialize an occupancy grid publisher (every time the localization is re-initialized, the map is published too)
    if (ros_group.check ("occupancygrid_topic"))
    {
        m_topic_occupancyGrid = ros_group.find ("occupancygrid_topic").asString();
        if (!m_rosPublisher_occupancyGrid.topic(m_topic_occupancyGrid))
        {
             if (m_rosNode)
             {
                 delete m_rosNode;
                 m_rosNode=0;
             }
             yCError(ROS_LOC) << "localizationModule: unable to publish data on " << m_topic_occupancyGrid << " topic, check your yarp-ROS network configuration";
             return false;
        }
    }

    //initialize a subscriber for pose particles
    if (ros_group.check("particles_topic"))
    {
        m_topic_particles = ros_group.find("particles_topic").asString();
        if (!m_rosSubscriber_particles.topic(m_topic_particles))
        {
            if (m_rosNode)
            {
                delete m_rosNode;
                m_rosNode = 0;
            }
            yCError(ROS_LOC) << "localizationModule: unable to subscribe data on " << m_topic_particles << " topic, check your yarp-ROS network configuration";
            return false;
        }
        yCDebug(ROS_LOC) << "opened " << m_topic_particles << " topic";
    }

     //initialize an initial pose publisher
    if (ros_group.check ("initialpose_topic")) 
    {
        m_topic_initial_pose = ros_group.find ("initialpose_topic").asString();
        {
            if (!m_rosPublisher_initial_pose.topic(m_topic_initial_pose))
            {
                if (m_rosNode)
                {
                    delete m_rosNode;
                    m_rosNode=0;
                }
                yCError(ROS_LOC) << "localizationModule: unable to publish data on " << m_topic_initial_pose << " topic, check your yarp-ROS network configuration";
                return false;
            }
        }
    }

    //map server group
    yCDebug(ROS_LOC) << map_group.toString();

    if (map_group.check("connect_to_yarp_mapserver") == false)
    {
        yCError(ROS_LOC) << "Missing `connect_to_yarp_mapserver` in [MAP] group";
        return false;
    }
    m_use_map_server= (map_group.find("connect_to_yarp_mapserver").asInt32()==1);

    //tf group
    if (tf_group.check("map_frame_id") == false)
    {
        yCError(ROS_LOC) << "Missing `map_frame_id` in [TF] group";
        return false;
    }
    if (tf_group.check("robot_frame_id") == false)
    {
        yCError(ROS_LOC) << "Missing `robot_frame_id` in [TF] group";
        return false;
    }
    m_frame_map_id = tf_group.find("map_frame_id").asString();
    m_frame_robot_id = tf_group.find("robot_frame_id").asString();


    //opens a client to receive localization data from transformServer
    Property options;
    options.put("device", "frameTransformClient");
    if(!tf_group.check("ft_client_config"))
    {
        yCWarning(ROS_LOC) << "Parameter 'ft_client_config' missing in [TF] group. Using default value: 'ftc_yarp_only.xml'";
        options.put("filexml_option", "ftc_yarp_only.xml");
    }
    else
    {
        options.put("filexml_option", tf_group.find("ft_client_config").asString());
    }
    if(!tf_group.check("ft_client_prefix"))
    {
        yCWarning(ROS_LOC) << "Parameter 'ft_client_prefix' missing in [TF] group. Using: '/" << m_name << "'";
        options.put("ft_client_prefix", m_name);
    }
    else
    {
        options.put("ft_client_prefix", tf_group.find("ft_client_prefix").asString());
    }

    if(!tf_group.check("ft_server_prefix"))
    {
        yCWarning(ROS_LOC) << "Parameter 'ft_server_prefix' missing in [TF] group. Using an empty string";
    }
    else
    {
        options.put("ft_server_prefix", tf_group.find("ft_server_prefix").asString());
    }

    if (m_ptf.open(options) == false)
    {
        yCError(ROS_LOC) << "Unable to open transform client";
        return false;
    }
    m_ptf.view(m_iTf);
    if (m_ptf.isValid() == false || m_iTf == 0)
    {
        yCError(ROS_LOC) << "Unable to view iTransform interface";
        return false;
    }

    if (m_use_map_server)
    {
        //opens a client to send/received data from mapServer
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", m_name); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", "/mapServer");
        if (m_pmap.open(map_options) == false)
        {
            yCWarning(ROS_LOC) << "Unable to open mapClient";
        }
        else
        {
            yCInfo(ROS_LOC) << "Opened mapClient";
            m_pmap.view(m_iMap);
            if (m_pmap.isValid() == false || m_iMap == 0)
            {
                yCError(ROS_LOC) << "Unable to view map interface";
                return false;
            }
        }
    }

    return true;
}

void rosLocalizerThread::threadRelease()
{
    if (m_ptf.isValid())
    {
        m_ptf.close();
    }
    if (m_pmap.isValid())
    {
        m_pmap.close();
    }

    if (m_rosPublisher_occupancyGrid.asPort().isOpen())
    {
        m_rosPublisher_occupancyGrid.interrupt();
        m_rosPublisher_occupancyGrid.close();
    }
    if (m_rosPublisher_initial_pose.asPort().isOpen())
    {
        m_rosPublisher_initial_pose.interrupt();
        m_rosPublisher_initial_pose.close();
    }
    if (m_rosSubscriber_particles.asPort().isOpen())
    {
        m_rosSubscriber_particles.interrupt();
        m_rosSubscriber_particles.close();
    }
    if (m_rosNode)
    {
        delete m_rosNode;
        m_rosNode = 0;
    }
}

bool rosLocalizer::open(yarp::os::Searchable& config)
{
    string cfg_temp = config.toString();
    Property p; p.fromString(cfg_temp);

    yCDebug(ROS_LOC) << "rosLocalizer configuration: \n" << p.toString().c_str();

    Bottle general_group = p.findGroup("ROSLOCALIZER_GENERAL");
    if (general_group.isNull()==false)
    {
        if (general_group.check("name")) {m_name = general_group.find("name").asString(); }
    }
    
    //create the thread
    thread = new rosLocalizerThread(0.010, m_name, p);

    //initial location initialization
    Bottle initial_group = p.findGroup("INITIAL_POS");
    if (initial_group.isNull())
    {
        yCError(ROS_LOC) << "Missing INITIAL_POS group!";
        return false;
    }
    if (initial_group.check("initial_x"))     { m_initial_loc.x = initial_group.find("initial_x").asFloat64(); }
    else { yCError(ROS_LOC) << "missing initial_x param"; return false; }
    if (initial_group.check("initial_y"))     { m_initial_loc.y = initial_group.find("initial_y").asFloat64(); }
    else { yCError(ROS_LOC) << "missing initial_y param"; return false; }
    if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asFloat64(); }
    else { yCError(ROS_LOC) << "missing initial_theta param"; return false; }
    if (initial_group.check("initial_map"))   { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
    else { yCError(ROS_LOC) << "missing initial_map param"; return false; }
    m_default_covariance_3x3.resize(3,3);
    m_default_covariance_3x3.zero();
    m_default_covariance_3x3[0][0] = 0.25;
    m_default_covariance_3x3[1][1] = 0.25;
    m_default_covariance_3x3[2][2] = 0.068538;
    setInitialPose(m_initial_loc,m_default_covariance_3x3);
    
    //start the thread
    if (!thread->start())
    {
        delete thread;
        return false;
    }

    bool ret = rpcPort.open(m_name+"/rpc");
    if (ret == false)
    {
        yCError(ROS_LOC) << "Unable to open module ports";
        return false;
    }

    rpcPortHandler.setInterface(this);
    rpcPort.setReader(rpcPortHandler);

    return true;
}

rosLocalizer::rosLocalizer()
{
    thread = NULL;
}

rosLocalizer::~rosLocalizer()
{
    if (thread)
    {
        delete thread;
        thread = NULL;
    }
}

bool rosLocalizer::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}

bool rosLocalizer::getCurrentPosition(Map2DLocation& loc, yarp::sig::Matrix& cov)
{
    yCWarning(ROS_LOC) << "Covariance matrix is not currently handled by rosLocalizer";
    thread->getCurrentLoc(loc);
    return true;
}

bool rosLocalizer::setInitialPose(const Map2DLocation& loc, const yarp::sig::Matrix& cov)
{
	if (thread)
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
		return thread->initializeLocalization(loc, cov6x6);
	}
	yCError(ROS_LOC) << "rosLocalizer thread not running";
	return false;
}

bool  rosLocalizer::startLocalizationService()
{
    if (thread)
    {
        return thread->startLoc();
    }
    return false;
}

bool  rosLocalizer::stopLocalizationService()
{
    if (thread)
    {
        return thread->stopLoc();
    }
    return false;
}
