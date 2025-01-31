/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <yarp/dev/ReturnValue.h>
#include <yarp/os/PeriodicThread.h>
#include <mutex>
#include <math.h>
#include <yarp/dev/IMap2D.h>
#include <localization_device_with_estimated_odometry.h>
#include "navigation_defines.h"

#ifndef POZYX_LOCALIZER_H
#define POZYX_LOCALIZER_H

using namespace yarp::os;

class pozyxLocalizer;
class pozyxLocalizerThread;

class pozyxLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    pozyxLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    pozyxLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(pozyxLocalizer* iface);
};

class pozyxLocalizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::ILocalization2D
{
public:
    pozyxLocalizerThread*    m_thread;
    pozyxLocalizerRPCHandler m_rpcPortHandler;
    yarp::os::Port           m_rpcPort;
    std::string              m_name = "/pozyxLocalizer";

public:
    virtual bool open(yarp::os::Searchable& config) override;

    pozyxLocalizer();
    virtual ~pozyxLocalizer();

    virtual bool close() override;

public:

    yarp::dev::ReturnValue getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    yarp::dev::ReturnValue getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    yarp::dev::ReturnValue getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    yarp::dev::ReturnValue setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    yarp::dev::ReturnValue getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    yarp::dev::ReturnValue startLocalizationService() override;
    yarp::dev::ReturnValue stopLocalizationService() override;
};

class pozyxLocalizerThread : public yarp::os::PeriodicThread,
                             public localization_device_with_estimated_odometry
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_map_to_pozyx_transform;
    yarp::dev::Nav2D::Map2DLocation     m_localization_data;
    yarp::dev::Nav2D::Map2DLocation     m_pozyx_data;
    std::vector<yarp::dev::Nav2D::Map2DLocation> m_anchors_pos;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;

    //publish anchors onto map as locations
    bool                         m_publish_anchors_as_map_locations;
    std::string                  m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    yarp::dev::PolyDriver        m_pMap;
    yarp::dev::Nav2D::IMap2D*    m_iMap = nullptr;
    std::string                  m_name = "/pozyxLocalizer";

private:
    bool publish_anchors_location();
    bool get_anchors_location();
    bool open_pozyx();

public:
    pozyxLocalizerThread(double _period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
};

#endif
