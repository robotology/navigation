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
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ReturnValue.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcClient.h>
#include <math.h>
#include <mutex>
#include <yarp/dev/IMap2D.h>
#include <localization_device_with_estimated_odometry.h>

#ifndef GAZEBO_LOCALIZER_H
#define GAZEBO_LOCALIZER_H

using namespace yarp::os;

class gazeboLocalizer;
class gazeboLocalizerThread;

class gazeboLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    gazeboLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    gazeboLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(gazeboLocalizer* iface);
};

class gazeboLocalizer : public yarp::dev::DeviceDriver,
                        public yarp::dev::Nav2D::ILocalization2D
{
public:
    gazeboLocalizerThread*    thread;
    gazeboLocalizerRPCHandler rpcPortHandler;
    yarp::os::Port            rpcPort;
    std::string               m_name = "/gazeboLocalizer";

public:
    virtual bool open(yarp::os::Searchable& config) override;

    gazeboLocalizer();
    virtual ~gazeboLocalizer();

    virtual bool close() override;

public:

    yarp::dev::ReturnValue  getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    yarp::dev::ReturnValue  getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    yarp::dev::ReturnValue  getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue  getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    yarp::dev::ReturnValue  setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue  getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue  setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue  startLocalizationService() override;
    yarp::dev::ReturnValue  stopLocalizationService() override;
};

class gazeboLocalizerThread : public yarp::os::PeriodicThread,
                              public localization_device_with_estimated_odometry
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_map_to_gazebo_transform;
    yarp::dev::Nav2D::Map2DLocation     m_localization_data;
    yarp::dev::Nav2D::Map2DLocation     m_gazebo_data;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_local_gazebo_port_name;
    std::string                  m_remote_gazebo_port_name;
    std::string                  m_object_name;
    yarp::os::RpcClient          m_port_gazebo_comm;
    std::string                  m_name;


private:
    bool open_gazebo();

public:
    gazeboLocalizerThread(double _period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
};

#endif
