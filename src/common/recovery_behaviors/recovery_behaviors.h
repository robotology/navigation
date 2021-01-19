/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#ifndef NAVIGATION_RECOVERY_BEHAVIOR_H
#define NAVIGATION_RECOVERY_BEHAVIOR_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Map2DLocation.h>
#include <yarp/dev/OdometryData.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IMap2D.h>
#include <mutex>
#include <math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

class generic_recovery_behaviour
{
    protected:
    yarp::os::BufferedPort<yarp::os::Bottle>* m_port_commands_output = nullptr;
    yarp::dev::IRangefinder2D* m_iLaser = nullptr;
    yarp::dev::Nav2D::ILocalization2D* m_iLoc = nullptr;
    yarp::dev::Nav2D::IMap2D* m_iMap = nullptr;

    protected:
    bool status_completed;

    public:
    virtual bool is_status_completed() = 0;
    virtual bool start_recovery_behavior() = 0;
    virtual bool stop_recovery_behavior() = 0;
};

class recovery_behavior : 
    public generic_recovery_behaviour,
    public yarp::os::PeriodicThread 
{
private:

    double ang_speed;
    double start_time;

private:
    //methods inherited from yarp::os::RateThread
    virtual void run() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;

public:
    recovery_behavior(yarp::os::BufferedPort<yarp::os::Bottle>* command_port,
                      yarp::dev::IRangefinder2D* iLas = nullptr,
                      yarp::dev::Nav2D::ILocalization2D* m_iLoc = nullptr,
                      yarp::dev::Nav2D::IMap2D* m_iMap = nullptr);
    virtual ~recovery_behavior() {};

    bool is_status_completed() override;
    bool start_recovery_behavior() override;
    bool stop_recovery_behavior() override;
};

#endif