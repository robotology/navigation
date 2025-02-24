/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include "navigation_defines.h"

#include <mutex>


using namespace yarp::os;

/**
 * \section localizationModule
 * A module which acts as the server side for a Localization2DClient.
 *
 *  Parameters required by this device are:
 * |      Parameter name     |         SubParameter        |   Type   |  Units  |     Default Value    |  Required  |                                          Description                                                                                             |       Notes         |
 * |:-----------------------:|:---------------------------:|:--------:|:-------:|:--------------------:|:----------:|:------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------:|
 * |  ROS2LOCALIZER_GENERAL  |  name                       |  string  |  -      |  localizationServer  |  Yes       |  The name of the module use to open ports                                                                                                        |                     |
 * |  INITIAL_POS            |  initial_x                  |  double  |  m      |  0.0                 |  Yes       |  Initial estimation of robot position                                                                                                            |  -                  |
 * |  INITIAL_POS            |  initial_y                  |  double  |  m      |  0.0                 |  Yes       |  Initial estimation of robot position                                                                                                            |  -                  |
 * |  INITIAL_POS            |  initial_theta              |  double  |  deg    |  0.0                 |  Yes       |  Initial estimation of robot position                                                                                                            |  -                  |
 * |  INITIAL_POS            |  initial_map                |  string  |  -      |  -                   |  Yes       |  Name of the map on which localization is performed                                                                                              |  -                  |
 * |  MAP                    |  connect_to_yarp_mapserver  |  int     |  0/1    |  -                   |  Yes       |  If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated                                                   |  -                  |
 * |  ROS2                   |  node_name                  |  string  |  -      |  -                   |  Yes       |  Name of the ROS2 node                                                                                                                           |  -                  |
 * |  ROS2                   |  initialpose_topic          |  string  |  -      |  -                   |  Yes       |  Name of the topic which will be used to publish the initial pose                                                                                |  -                  |
 * |  ROS2                   |  currentpose_topic          |  string  |  -      |  -                   |  Yes       |  Name of the topic to subscribe to in order to receive the current position from ROS2                                                            |  -                  |
 * |  ROS2                   |  occupancygrid_topic        |  string  |  -      |  -                   |  Yes       |  Name of the topic which will be used to publish map data when initial pose is updated                                                           |  -                  |
 * |  ROS2                   |  particles_topic            |  string  |  -      |  -                   |  Yes       |  Name of the topic from which particles data will be received                                                                                    |  -                  |
 * |  TF                     |  map_frame_id               |  string  |  -      |  -                   |  Yes       |  Name of the map reference frame                                                                                                                 |  e.g. /map          |
 * |  TF                     |  robot_frame_id             |  string  |  -      |  -                   |  Yes       |  Name of the robot reference frame                                                                                                               |  e.g. /mobile_base  |
 * |  TF                     |  ft_client_config           |  string  |  -      |  ftc_yarp_only.xml   |  No        |  The name of the xml file containing the needed client configuration                                                                             |  -                  |
 * |  TF                     |  ft_client_prefix           |  string  |  -      |  ""                  |  No        |  A prefix to add to the names of all the ports opened by the frameTransformClient                                                                |  -                  |
 * |  TF                     |  ft_server_prefix           |  string  |  -      |  ""                  |  No        |  The prefix added to all the names of the ports opened by the frameTransformServer                                                               |  -                  |
 * |  LOCALIZATION           |  localization_mode          |  string  |  -      |  -                   |  Yes       |  If set to "ros", the module will use a ros topic to receive localization data. If set to 'tf' it will use data received on the transformClient  |  -                  |
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
    ros2LocalizerThread*    m_thread;
    ros2LocalizerRPCHandler m_rpcPortHandler;
    yarp::os::Port          m_rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    ros2Localizer();
    virtual ~ros2Localizer();

    virtual bool close() override;

    yarp::dev::ReturnValue   getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    yarp::dev::ReturnValue   getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    yarp::dev::ReturnValue   getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    yarp::dev::ReturnValue   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue   startLocalizationService() override;
    yarp::dev::ReturnValue   stopLocalizationService() override;
};

class ros2LocalizerThread : public yarp::os::PeriodicThread,
                           public localization_device_with_estimated_odometry
{
protected:
    //general
    std::string                      m_name;
    std::string                      m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    double                           m_last_statistics_printed;
    double                           m_last_published_map;
    yarp::dev::Nav2D::MapGrid2D      m_current_map;
    yarp::dev::Nav2D::Map2DLocation  m_localization_data;
    std::mutex                       m_mutex;
    yarp::os::Searchable&            m_cfg;


    //configuration options
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
    std::string                       m_topic_currpose;
    rclcpp::Node::SharedPtr           m_node{nullptr};
    Ros2Spinner*                      m_innerSpinner{nullptr};
    enum { use_tf_loc = 0, use_ros_loc=1, use_unknown=-1 }                          m_loc_mode ;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr     m_ros2Publisher_initial_pose{nullptr};
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  m_ros2Subscriber_current_pose{nullptr};
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr                      m_ros2Publisher_occupancyGrid{nullptr};
    rclcpp::Subscription<nav2_msgs::msg::ParticleCloud>::SharedPtr                  m_ros2Subscriber_particles{nullptr};
    nav2_msgs::msg::ParticleCloud                                                   m_last_received_particles;

public:
    ros2LocalizerThread(double period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
    void publish_map();
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& roscov6x6);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses);
    void particles_callback(const nav2_msgs::msg::ParticleCloud msg);
    void currPose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped inputPose);
    bool startLoc();
    bool stopLoc();
    builtin_interfaces::msg::Time ros2TimeFromYarpNow();
};

#endif  // ROS2LOCALIZER_H

