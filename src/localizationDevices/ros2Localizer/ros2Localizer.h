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

#ifndef ROS2LOCALIZER_H
#define ROS2LOCALIZER_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/os/Subscriber.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IFrameTransform.h>
#include <mutex>
#include <math.h>


#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <nav_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h>
//#include <nav_msgs/msg/rosidl_typesupport_connext_cpp__visibility_control.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/msg/particle_cloud.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <yarp/math/Math.h>
#include <localization_device_with_estimated_odometry.h>
#include "Ros2Spinner.h"
#include "Ros2Utils.h"
#include "Ros2Subscriber.h"
#include "navigation_defines.h"

#include <mutex>


using namespace yarp::os;

/**
 * \section localizationModule
 * A module which acts as the server side for a Localization2DClient.
 *
 *  Parameters required by this device are:
 * |      Parameter name     |              SubParameter             |   Type   |  Units  |     Default Value    |  Required  |                                          Description                                            |                            Notes                           |
 * |:-----------------------:|:-------------------------------------:|:--------:|:-------:|:--------------------:|:----------:|:-----------------------------------------------------------------------------------------------:|:----------------------------------------------------------:|
 * |  ROS2LOCALIZER_GENERAL  |  name                                 |  string  |  -      |  localizationServer  |  Yes       |  The name of the module use to open ports                                                       |                                                            |
 * |  INITIAL_POS            |  initial_x                            |  double  |  m      |  0.0                 |  Yes       |  Initial estimation of robot position                                                           |  -                                                         |
 * |  INITIAL_POS            |  initial_y                            |  double  |  m      |  0.0                 |  Yes       |  Initial estimation of robot position                                                           |  -                                                         |
 * |  INITIAL_POS            |  initial_theta                        |  double  |  deg    |  0.0                 |  Yes       |  Initial estimation of robot position                                                           |  -                                                         |
 * |  INITIAL_POS            |  initial_map                          |  string  |  -      |  -                   |  Yes       |  Name of the map on which localization is performed                                             |  -                                                         |
 * |  MAP                    |  connect_to_yarp_mapserver            |  int     |  0/1    |  -                   |  Yes       |  If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated  |  -                                                         |
 * |  ROS2                   |  node_name                            |  string  |  -      |  -                   |  Yes       |  Name of the ROS2 node                                                                          |  -                                                         |
 * |  ROS2                   |  initialpose_topic                    |  string  |  -      |  -                   |  Yes       |  Name of the topic which will be used to publish the initial pose                               |  -                                                         |
 * |  ROS2                   |  occupancygrid_topic                  |  string  |  -      |  -                   |  Yes       |  Name of the topic which will be used to publish map data when initial pose is updated          |  -                                                         |
 * |  ROS2                   |  particles_topic                      |  string  |  -      |  -                   |  Yes       |  Name of the topic from which particles data will be received                                   |  -                                                         |
 * |  TF                     |  map_frame_id                         |  string  |  -      |  -                   |  Yes       |  Name of the map reference frame                                                                |  e.g. /map                                                 |
 * |  TF                     |  robot_frame_id                       |  string  |  -      |  -                   |  Yes       |  Name of the robot reference frame                                                              |  e.g. /mobile_base                                         |
 * |  LOCALIZATION           |  use_localization_from_odometry_port  |  int     |  0/1    |  -                   |  Yes       |  If set to 1, the module will use a port to receive localization data                           |  Incompatible with 'use_localization_from_tf=1'            |
 * |  LOCALIZATION           |  use_localization_from_tf             |  int     |  0/1    |  -                   |  Yes       |  If set to 1, the module will use a tfClient to receive localization data                       |  Incompatible with 'use_localization_from_odometry_port=1  |
 */

class ros2Localizer;
class ros2LocalizerThread;

class ros2LocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    ros2Localizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    ros2LocalizerRPCHandler() : interface(NULL) { }
    void setInterface(ros2Localizer* iface);
};

class ros2Localizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::ILocalization2D
{
private:
    yarp::sig::Matrix                m_default_covariance_3x3;
    yarp::dev::Nav2D::Map2DLocation  m_initial_loc;
    std::string                      m_name= "/ros2Localizer";

public:
    ros2LocalizerThread*    thread;
    ros2LocalizerRPCHandler rpcPortHandler;
    yarp::os::Port         rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    ros2Localizer();
    virtual ~ros2Localizer();

    virtual bool close() override;

public:

    bool   getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    bool   getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    bool   getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    bool   startLocalizationService() override;
    bool   stopLocalizationService() override;
};

class ros2LocalizerThread : public yarp::os::PeriodicThread,
                           public localization_device_with_estimated_odometry
{
protected:
    //general
    std::string                      m_name;
    double                           m_last_statistics_printed;
    double                           m_last_published_map;
    yarp::dev::Nav2D::MapGrid2D      m_current_map;
    yarp::dev::Nav2D::Map2DLocation  m_localization_data;
    std::mutex                       m_mutex;
    yarp::os::Searchable&            m_cfg;
    Bottle                           ros_group;


    //configuration options
    bool                         m_use_localization_from_odometry_port;
    bool                         m_use_localization_from_tf;
    bool                         m_use_map_server;
    bool                         m_spun{false};

    //tf data
    yarp::dev::PolyDriver        m_ptf;
    yarp::dev::IFrameTransform*  m_iTf;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_map_id;

    //map interface 
    yarp::dev::PolyDriver        m_pmap;
    yarp::dev::Nav2D::IMap2D*    m_iMap;

    //ROS2
    size_t                            m_seq_counter;
    builtin_interfaces::msg::Time     m_rosTime;
    std::string                       m_nodeName;
    std::string                       m_topic_initial_pose;
    std::string                       m_topic_occupancyGrid;
    std::string                       m_topic_particles;
    rclcpp::Node::SharedPtr           m_node{nullptr};
    Ros2Spinner*                      m_innerSpinner{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  m_ros2Publisher_initial_pose{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr                   m_ros2Publisher_occupancyGrid{nullptr};
    rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr               m_ros2Subscriber_particles{nullptr};
    nav2_msgs::msg::ParticleCloud                                                m_last_received_particles;

public:
    ros2LocalizerThread(double period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
    void publish_map();

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& roscov6x6);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses);
    void particles_callback(const nav2_msgs::msg::ParticleCloud msg);
    bool startLoc();
    bool stopLoc();
    builtin_interfaces::msg::Time ros2TimeFromYarpNow();
};

#endif  // ROS2LOCALIZER_H

