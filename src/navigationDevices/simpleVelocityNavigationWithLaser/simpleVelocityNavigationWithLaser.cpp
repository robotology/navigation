/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <math.h>
#include <limits>
#include <navigation_defines.h>
#include "simpleVelocityNavigationWithLaser.h"

//the following ugly definition is just a reminder to check the system behavior when a method returns true (with invalid data) or false.
#define NOT_IMPLEMENTED false

using namespace yarp::os;
using namespace yarp::dev;

simpleVelocityNavigationWithLaser::simpleVelocityNavigationWithLaser() : simpleVelocityNavigation()
{
    m_localName = "simpleVelocityNavigationWithLaser";
    m_obstacles_handler = nullptr;
}

bool simpleVelocityNavigationWithLaser::open(yarp::os::Searchable& config)
{
    //open the laser interface
    Bottle laserBottle = config.findGroup("LASER");

    if (laserBottle.isNull())
    {
        yError("LASER group not found,closing");
        return false;
    }

    if (laserBottle.check("laser_port") == false)
    {
        yError("laser_port param not found,closing");
        return false;
    }
    std::string laser_remote_port = laserBottle.find("laser_port").asString();

    //opens the laser client and the corresponding interface
    Property options;
    options.put("device", "Rangefinder2DClient");
    options.put("local", std::string("/")+m_localName+"/laser:i");
    options.put("remote", laser_remote_port);
    if (m_pLas.open(options) == false)
    {
        yError() << "Unable to open laser driver";
        return false;
    }
    m_pLas.view(m_iLaser);
    if (m_iLaser == 0)
    {
        yError() << "Unable to open laser interface";
        return false;
    }

    m_obstacles_handler = new obstacles_class(config);

    bool b = simpleVelocityNavigation::open(config);
    return b;
}

bool simpleVelocityNavigationWithLaser::close()
{
    bool b = simpleVelocityNavigation::close();

    if (m_pLas.isValid()) m_pLas.close();

    if (m_obstacles_handler)
    {
        delete m_obstacles_handler;
        m_obstacles_handler = nullptr;
    }
    return b;
}

bool simpleVelocityNavigationWithLaser::threadInit()
{
    return  simpleVelocityNavigation::threadInit();
}

void simpleVelocityNavigationWithLaser::threadRelease()
{
    simpleVelocityNavigation::threadRelease();
}

void simpleVelocityNavigationWithLaser::run()
{
    simpleVelocityNavigation::run();
}

void simpleVelocityNavigationWithLaser::send_command(control_type control_data)
{
    //check the obstacle presence here
    std::vector<yarp::dev::LaserMeasurementData> laser_data;
    m_iLaser->getLaserMeasurement(laser_data);

    //if obstacles are present, override robot velocity controls
    if (m_obstacles_handler->check_obstacles_in_path(laser_data))
    {
        yWarning() << "Obstacles detected, robot is waiting!";
        if (m_navigation_status == yarp::dev::navigation_status_moving)
        {
            m_navigation_status == yarp::dev::navigation_status_waiting_obstacle;
        }
    }

    static yarp::os::Stamp stamp;
    stamp.update();
    Bottle &b = m_port_commands_output.prepare();
    m_port_commands_output.setEnvelope(stamp);
    b.clear();
    b.addInt(BASECONTROL_COMMAND_VELOCIY_CARTESIAN);
    b.addDouble(control_data.linear_xvel);    // lin_vel in m/s
    b.addDouble(control_data.linear_yvel);    // lin_vel in m/s
    b.addDouble(control_data.angular_vel);    // ang_vel in deg/s
    b.addDouble(100);
    m_port_commands_output.write();
}

