/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef IKART_MOTORS_CTRL_H
#define IKART_MOTORS_CTRL_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>
#include "../motors.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class iKart_MotorControl : public MotorControl
{
    //robot geometry
    double              geom_r;
    double              geom_L;
    double              g_angle;

public:
    /**
    * Constructor
    * @param _driver is a pointer to a remoteControlBoard driver.
    */
    iKart_MotorControl(PolyDriver* _driver);

    /**
    * Destructor
    */
    ~iKart_MotorControl();

    //The following methods are documented in base class odometry.h
    bool open(const Property &_options);
    void execute_none();
    void execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void close();
    void set_motors_filter(filter_frequency b) {motors_filter_enabled=b;}
    double get_vlin_coeff();
    double get_vang_coeff();
};

#endif
