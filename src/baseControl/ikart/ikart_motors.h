/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
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
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>
#include "../motors.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#define DEFAULT_MAX_LINEAR_VEL  0.42  // maximum linear  velocity (m/s)
#define DEFAULT_MAX_ANGULAR_VEL 24.0  // maximum angular velocity (deg/s)

class iKart_MotorControl : public MotorControl
{
public:
    iKart_MotorControl(unsigned int _period, PolyDriver* _driver);
    ~iKart_MotorControl();
    bool set_control_velocity();
    bool set_control_openloop();
    bool set_control_idle();

    bool open(ResourceFinder &_rf, Property &_options);
    void execute_none();
    void execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed);
    void close();
    bool check_motors_on();
    void updateControlMode();
    void printStats();
    void set_motors_filter(int b) {motors_filter_enabled=b;}
 
    double get_max_linear_vel()   {return max_linear_vel;}
    double get_max_angular_vel()  {return max_angular_vel;}
};

#endif
