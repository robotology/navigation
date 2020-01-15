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
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IRangefinder2D.h>
#include <math.h>
#include "../simpleVelocityNavigation/simpleVelocityNavigation.h"
#include <obstacles.h>

#ifndef NAV_SIMPLE_VEL_NAV_WITH_LASER_H
#define NAV_SIMPLE_VEL_NAV_WITH_LASER_H

#define DEFAULT_THREAD_PERIOD 0.02 //s

class simpleVelocityNavigationWithLaser :
                                 public simpleVelocityNavigation
                                 //public yarp::os::PeriodicThread,
                                 //public yarp::dev::INavigation2DTargetActions,
                                 //public yarp::dev::INavigation2DControlActions,
                                 //public yarp::dev::DeviceDriver
{
protected:
    //laser stuff
    yarp::dev::PolyDriver                      m_pLas;
    yarp::dev::IRangefinder2D*                 m_iLaser;

    //obstacles stuff
    obstacles_class*                           m_obstacles_handler;

public:
    simpleVelocityNavigationWithLaser();

protected:
    virtual void send_command(control_type control_data);

public:
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

};

#endif
