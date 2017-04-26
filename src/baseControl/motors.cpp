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

#include "motors.h"
#include "filters.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

void MotorControl::close()
{
}

MotorControl::~MotorControl()
{
    close();
}

void  MotorControl::apply_motor_filter(int i)
{
    if (motors_filter_enabled == 1)
    {
        F[i] = control_filters::lp_filter_1Hz(F[i], i);
    }
    else if (motors_filter_enabled == 2)
    {
        F[i] = control_filters::lp_filter_2Hz(F[i], i);
    }
    else if (motors_filter_enabled == 4) //default
    {
        F[i] = control_filters::lp_filter_4Hz(F[i], i);
    }
    else if (motors_filter_enabled == 8)
    {
        F[i] = control_filters::lp_filter_8Hz(F[i], i);
    }
}

bool MotorControl::open(ResourceFinder &_rf, Property &_options)
{
    ctrl_options = _options;
    localName = ctrl_options.find("local").asString();

    if (_rf.check("no_motors_filter"))
    {
        yInfo("'no_filter' option found. Turning off PWM filter.");
        motors_filter_enabled=0;
    }

    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ivel);
    ok = ok & control_board_driver->view(ienc);
    ok = ok & control_board_driver->view(ipwm);
    ok = ok & control_board_driver->view(ipid);
    ok = ok & control_board_driver->view(iamp);
    ok = ok & control_board_driver->view(icmd);
    if(!ok)
    {
        yError("One or more devices has not been viewed, returning\n");
        return false;
    }

    if (!ctrl_options.check("GENERAL"))
    {
        yError() << "Missing [GENERAL] section";
        return false;
    }
    yarp::os::Bottle& general_options = ctrl_options.findGroup("GENERAL");

    motors_filter_enabled = general_options.check("motors_filter_enabled", Value(4), "motors filter frequency (1/2/4/8Hz, 0 = disabled)").asInt();

    if (!general_options.check("max_linear_vel"))
    {
        yError("Error reading from .ini file, missing, max_linear_vel parameter, section GENERAL");
        return false;
    }
    if (!general_options.check("max_angular_vel"))
    {
        yError("Error reading from .ini file, missing, max_angular_vel parameter, section GENERAL");
        return false;
    }

    double tmp = 0;
    tmp = (general_options.check("max_angular_vel", Value(0), "maximum angular velocity of the platform [deg/s]")).asDouble();
    if (tmp>0 && tmp < DEFAULT_MAX_ANGULAR_VEL) max_angular_vel = tmp;
    tmp = (general_options.check("max_linear_vel", Value(0), "maximum linear velocity of the platform [m/s]")).asDouble();
    if (tmp>0 && tmp < DEFAULT_MAX_LINEAR_VEL) max_linear_vel = tmp;

    localName = ctrl_options.find("local").asString();

    return true;
}

MotorControl::MotorControl(unsigned int _period, PolyDriver* _driver)
{
    control_board_driver = _driver;

    thread_timeout_counter = 0;

    max_linear_vel = DEFAULT_MAX_LINEAR_VEL;
    max_angular_vel = DEFAULT_MAX_ANGULAR_VEL;

    thread_period = _period;
}
