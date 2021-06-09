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
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/OdometryData.h>
#include <yarp/os/PeriodicThread.h>
#include <mutex>
#include <math.h>
#include <localization_device_with_estimated_odometry.h>

using namespace yarp::os;

/**
 * \section odomLocalizer
 * A localization device which can be wrapped by a Localization2DServer.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------------------------------------------------------------:|:-----:|
 * | ODOMLOCALIZER_GENERAL  |  name  | string  | -              | localizationServer | Yes          | The name of the module use to open ports                          |       |
 * | INITIAL_POS    |  initial_x     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_y     | double  | m              | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_theta | double  | deg            | 0.0                | Yes          | Initial estimation of robot position                              | -     |
 * | INITIAL_POS    |  initial_map   | string  | -              |   -                | Yes          | Name of the map on which localization is performed                | -     |
 * | ODOMETRY       |  odometry_broadcast_port       | string   |  -   |   -         | Yes          | Full name of port broadcasting the localization data. The server will connect to this port.  | -    |
 */

class odomLocalizer;
class odomLocalizerThread;

class odomLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    odomLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    odomLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(odomLocalizer* iface);
};

class odomLocalizer : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::ILocalization2D
{
public:
    odomLocalizerThread*    m_thread;
    odomLocalizerRPCHandler m_rpcPortHandler;
    yarp::os::Port          m_rpcPort;
    std::string             m_name = "/odomLocalizer";

public:
    virtual bool open(yarp::os::Searchable& config) override;

    odomLocalizer();
    virtual ~odomLocalizer();

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

class odomLocalizerThread : public yarp::os::PeriodicThread,
                            public localization_device_with_estimated_odometry
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_initial_loc;
    yarp::dev::Nav2D::Map2DLocation     m_initial_odom;
    yarp::dev::Nav2D::Map2DLocation     m_current_loc;
    yarp::dev::Nav2D::Map2DLocation     m_current_odom;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_name;

    //odometry port
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::dev::OdometryData>  m_port_odometry_input;
    double                       m_last_port_odometryData_time;
    yarp::dev::OdometryData      m_last_port_odometryData;

public:
    odomLocalizerThread(const double _period, std::string _name, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

    bool   getLastOdometryDataFromPort(yarp::dev::OdometryData& odom);

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
};
