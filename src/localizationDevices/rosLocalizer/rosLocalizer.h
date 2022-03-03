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
#include <yarp/os/Subscriber.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/rosmsg/geometry_msgs/PoseStamped.h>
#include <yarp/rosmsg/geometry_msgs/PoseWithCovarianceStamped.h>
#include <yarp/rosmsg/nav_msgs/OccupancyGrid.h>
#include <yarp/rosmsg/geometry_msgs/PoseArray.h>
#include <mutex>
#include <math.h>

#include <localization_device_with_estimated_odometry.h>
#include "navigation_defines.h"

using namespace yarp::os;

/**
 * \section localizationModule
 * A module which acts as the server side for a Localization2DClient.
 *
 *  Parameters required by this device are:
 * | Parameter name       | SubParameter                         | Type    | Units  | Default Value      | Required     | Description                                                                                    | Notes                                                    |
 * |:--------------------:|:------------------------------------:|:-------:|:------:|:------------------:|:-----------: |:----------------------------------------------------------------------------------------------:|:--------------------------------------------------------:|
 * | ROSLOCALIZER_GENERAL |  name                                | string  | -      | localizationServer | Yes          | The name of the module use to open ports                                                       |                                                          |
 * | ROSLOCALIZER_GENERAL |  enable_ros                          | int     | 0/1    | -                  | Yes          | If set to 1, the module will open the ROS topic specified by ROS::initialpose_topic parameter  |                                                          |
 * | INITIAL_POS          | initial_x                            | double  | m      | 0.0                | Yes          | Initial estimation of robot position                                                           | -                                                        |
 * | INITIAL_POS          | initial_y                            | double  | m      | 0.0                | Yes          | Initial estimation of robot position                                                           | -                                                        |
 * | INITIAL_POS          | initial_theta                        | double  | deg    | 0.0                | Yes          | Initial estimation of robot position                                                           | -                                                        |
 * | INITIAL_POS          | initial_map                          | string  | -      | -                  | Yes          | Name of the map on which localization is performed                                             | -                                                        |
 * | MAP                  | connect_to_yarp_mapserver            | int     | 0/1    | -                  | Yes          | If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated  | -                                                        |
 * | ROS                  | initialpose_topic                    | string  | -      | -                  | No           | Name of the topic which will be used to publish the initial pose                               | -                                                        |
 * | ROS                  | occupancygrid_topic                  | string  | -      | -                  | No           | Name of the topic which will be used to publish map data when initial pose is updated          | -                                                        |
 * | TF                   | map_frame_id                         | string  | -      | -                  | Yes          | Name of the map reference frame                                                                | e.g. /map                                                |
 * | TF                   | robot_frame_id                       | string  | -      | -                  | Yes          | Name of the robot reference frame                                                              | e.g. /mobile_base                                        |
 * | TF                   | ft_client_config                     | string  | -      | ftc_yarp_only.xml  | No           | The name of the xml file containing the needed client configuration                            | -                                                        |
 * | TF                   | ft_client_prefix                     | string  | -      | ""                 | No           | A prefix to add to the names of all the ports opened by the frameTransformClient               | -                                                        |
 * | TF                   | ft_server_prefix                     | string  | -      | ""                 | No           | The prefix added to all the names of the ports opened by the frameTransformServer              | -                                                        |
 * | LOCALIZATION         | use_localization_from_ros            | int     | 0/1    | -                  | Yes          | If set to 1, the module will use a ros topic to receive localization data                      | Incompatible with 'use_localization_from_tf=1'           |
 * | LOCALIZATION         | use_localization_from_tf             | int     | 0/1    | -                  | Yes          | If set to 1, the module will use a tfClient to receive localization data                       | Incompatible with 'use_localization_from_odometry_port=1 |
 */

class rosLocalizer;
class rosLocalizerThread;

class rosLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    rosLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    rosLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(rosLocalizer* iface);
};

class rosLocalizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::ILocalization2D
{
private:
    yarp::sig::Matrix                m_default_covariance_3x3;
    yarp::dev::Nav2D::Map2DLocation  m_initial_loc;
    std::string                      m_name= "/rosLocalizer";

public:
    rosLocalizerThread*    thread;
    rosLocalizerRPCHandler rpcPortHandler;
    yarp::os::Port         rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    rosLocalizer();
    virtual ~rosLocalizer();

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

class rosLocalizerThread : public yarp::os::PeriodicThread,
                           public localization_device_with_estimated_odometry
{
protected:
    //general
    std::string                  m_name;
    std::string                  m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    double                       m_last_statistics_printed;
    double                       m_last_published_map;
    yarp::dev::Nav2D::MapGrid2D         m_current_map;
    yarp::dev::Nav2D::Map2DLocation     m_localization_data;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;

    //configuration options
    bool                         m_ros_enabled;
    bool                         m_use_localization_from_odometry_port;
    bool                         m_use_localization_from_tf;
    bool                         m_use_map_server;

    //tf data
    yarp::dev::PolyDriver        m_ptf;
    yarp::dev::IFrameTransform*  m_iTf;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_map_id;

    //map interface 
    yarp::dev::PolyDriver        m_pmap;
    yarp::dev::Nav2D::IMap2D*    m_iMap;

    //ROS
    size_t                            m_seq_counter;
    yarp::rosmsg::TickTime            m_rosTime; 
    yarp::os::Node*                   m_rosNode;
    std::string                       m_topic_initial_pose;
    std::string                       m_topic_occupancyGrid;
    std::string                       m_topic_particles;
    std::string                       m_topic_currpose;
    enum { use_tf_loc = 0, use_ros_loc=1, use_unknown=-1 } m_loc_mode ;
    yarp::os::Publisher<yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped> m_rosPublisher_initial_pose;
    yarp::os::Publisher<yarp::rosmsg::nav_msgs::OccupancyGrid> m_rosPublisher_occupancyGrid;
    yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::PoseArray> m_rosSubscriber_particles;
    yarp::os::Subscriber <yarp::rosmsg::geometry_msgs::PoseWithCovarianceStamped> m_rosSubscriber_current_pose;
    yarp::rosmsg::geometry_msgs::PoseArray m_last_received_particles;

public:
    rosLocalizerThread(double _period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
    void publish_map();

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& roscov6x6);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses);
    bool startLoc();
    bool stopLoc();
};
