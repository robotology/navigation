/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
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
#include <math.h>
#include <mutex>
#include <yarp/dev/IMap2D.h>
#include <movable_localization_device.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

//realsense
#include <librealsense2/rs.hpp>

#ifndef T265_LOCALIZER_H
#define T265_LOCALIZER_H

using namespace yarp::os;

class t265Localizer;
class t265LocalizerThread;

class t265LocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    t265Localizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    t265LocalizerRPCHandler() : interface(NULL) { }
    void setInterface(t265Localizer* iface);
};

class t265Localizer : public yarp::dev::DeviceDriver,
                      public yarp::dev::Nav2D::ILocalization2D
{
public:
    t265LocalizerThread*    thread;
    t265LocalizerRPCHandler rpcPortHandler;
    yarp::os::Port          rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    t265Localizer();
    virtual ~t265Localizer();

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

class odometry_handler : public BufferedPort<yarp::dev::OdometryData>
{
public:

    rs2::wheel_odometer          m_rs_odometry_handler;
    rs2_vector                   m_linear_velocity;
    size_t                       m_counter;

    odometry_handler(const rs2::device& dev);

    using BufferedPort<yarp::dev::OdometryData>::onRead;
    void onRead(yarp::dev::OdometryData& b) override;
};

class t265LocalizerThread : public yarp::os::PeriodicThread,
                                   movable_localization_device
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_map_to_device_transform;
    std::mutex                   m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_local_name;
    std::string                  m_local_name_prefix;

    yarp::dev::Nav2D::Map2DLocation     m_initial_loc;
    yarp::dev::Nav2D::Map2DLocation     m_initial_device_data;
    yarp::dev::Nav2D::Map2DLocation     m_current_loc;
    yarp::dev::OdometryData             m_current_odom;
    yarp::dev::Nav2D::Map2DLocation     m_current_device_data;

    //velocity estimation
    yarp::sig::Vector            m_odom_vel;
    yarp::sig::Vector            m_robot_vel;
    iCub::ctrl::AWLinEstimator*  m_estimator;

    //map server
    std::string                  m_remote_map;
    yarp::dev::PolyDriver        m_pMap;
    yarp::dev::Nav2D::IMap2D*    m_iMap;

    //device
    rs2::pipeline                m_realsense_pipe;
    rs2::config                  m_realsense_cfg;
    odometry_handler*            m_odometry_handler;

private:
    bool open_device();

public:
    t265LocalizerThread(double _period, yarp::os::Searchable& _cfg);
    ~t265LocalizerThread();
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentOdom(yarp::dev::OdometryData& odom);
    void odometry_update();
};

#endif
