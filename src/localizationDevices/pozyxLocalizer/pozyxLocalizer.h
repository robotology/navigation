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
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
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

    bool   getLocalizationStatus(yarp::dev::Nav2D::LocalizationStatusEnum& status) override;
    bool   getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   getEstimatedOdometry(yarp::dev::OdometryData& odom) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;
    bool   startLocalizationService() override;
    bool   stopLocalizationService() override;
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
    std::string                  m_remote_map;
    yarp::dev::PolyDriver        m_pMap;
    yarp::dev::Nav2D::IMap2D*    m_iMap;
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
