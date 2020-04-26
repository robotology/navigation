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
#include <mutex>
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
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IFrameTransform.h>

#include <math.h>

using namespace yarp::os;

/**
 * \section localizationModule
 * A module which acts as the server side for a Localization2DClient.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------------------------------------------------------------:|:-----:|
 * | GENERAL        |  module_name   | string  | -              | localizationServer | Yes          | The name of the module use to open ports                          |       |
 * | GENERAL        |  enable_ros    | int     | 0/1            | -                  | Yes          | If set to 1, the module will open the ROS topic specified by ROS::initialpose_topic parameter     |       |
 * | INITIAL_POS    |  initial_x     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_y     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_theta | double  | deg            | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_map   | string  | -              |   -                | Yes          | Name of the map on which localization is performed                | -     |
 * | MAP            |  connect_to_yarp_mapserver   | int  | 0/1     |   -              | Yes          | If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated | -     |
 * | ROS            |  initialpose_topic   | string     | -         |   -              | No           | Name of the topic which will be used to publish the initial pose  | -     |
 * | ROS            |  occupancygrid_topic | string     | -         |   -              | No           | Name of the topic which will be used to publish map data when initial pose is updated         | -     |
 * | TF             |  map_frame_id      | string     | -         |   -                | Yes          | Name of the map reference frame                                   | e.g. /map    |
 * | TF             |  robot_frame_id    | string     | -         |   -                | Yes          | Name of the robot reference frame                                 | e.g. /mobile_base    |
 * | LOCALIZATION   |  use_localization_from_odometry_port    | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a port to receive localization data                         | Incompatible with 'use_localization_from_tf=1'  |
 * | LOCALIZATION   |  use_localization_from_tf      | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a tfClient to receive localization data                     | Incompatible with 'use_localization_from_odometry_port=1 |
 */

#ifndef ISAAC_LOCALIZER_H
#define ISAAC_LOCALIZER_H

class isaacLocalizer;
class isaacLocalizerThread;

class isaacLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    isaacLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    isaacLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(isaacLocalizer* iface);
};

class isaacLocalizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::ILocalization2D
{
public:
    isaacLocalizerThread*    thread;
    isaacLocalizerRPCHandler rpcPortHandler;
    yarp::os::Port         rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    isaacLocalizer();
    virtual ~isaacLocalizer();

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

class isaacLocalizerThread : public yarp::os::PeriodicThread
{
protected:
    //general
    std::string                  m_module_name;
    double                       m_last_statistics_printed;
    double                       m_last_published_map;
    yarp::dev::Nav2D::MapGrid2D        m_current_map;
    yarp::dev::Nav2D::Map2DLocation    m_initial_loc;
    yarp::dev::Nav2D::Map2DLocation    m_localization_data;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_local_name;

    //ISAAC
    yarp::os::BufferedPort<yarp::dev::Nav2D::Map2DLocation> m_port_localization_data;

public:
    isaacLocalizerThread(double _period, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
    void publish_map();

public:
    bool initializeLocalization(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
};

#endif
