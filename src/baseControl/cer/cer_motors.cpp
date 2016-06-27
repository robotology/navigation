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

#include "cer_motors.h"
#include "../filters.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

bool CER_MotorControl::set_control_openloop()
{
    yInfo ("Setting openloop mode");
    icmd->setOpenLoopMode(0);
    icmd->setOpenLoopMode(1);
    iopl->setRefOutput(0,0);
    iopl->setRefOutput(1,0);
    return true;
}

bool CER_MotorControl::set_control_velocity()
{
    yInfo ("Setting velocity mode");
    icmd->setVelocityMode(0);
    icmd->setVelocityMode(1);
    ivel->setRefAcceleration(0, 1000000);
    ivel->setRefAcceleration(1, 1000000);
    ivel->velocityMove(0,0);
    ivel->velocityMove(1,0);
    return true;
}

bool CER_MotorControl::set_control_idle()
{
    yInfo ("Setting ilde mode");
    icmd->setControlMode(0, VOCAB_CM_IDLE);
    icmd->setControlMode(1, VOCAB_CM_IDLE);
    icmd->setControlMode(2, VOCAB_CM_IDLE);
    icmd->setControlMode(3, VOCAB_CM_IDLE);
    yInfo("Motors now off");
    return true;
}

bool CER_MotorControl::check_motors_on()
{
    int c0(0),c1(0),c2(0);
    yarp::os::Time::delay(0.05);
    icmd->getControlMode(0,&c0);
    icmd->getControlMode(0,&c1);
    if (c0 != VOCAB_CM_IDLE && c1 != VOCAB_CM_IDLE)
    {
        yInfo("Motors now on\n");
        return true;
    }
    else
    {
        yInfo("Unable to turn motors on! fault pressed?\n");
        return false;
    }
}

void CER_MotorControl::updateControlMode()
{
    icmd->getControlMode(0, &board_control_modes[0]);
    icmd->getControlMode(1, &board_control_modes[1]);
    /*
        for (int i=0; i<3; i++)
        if (board_control_modes[i]==VOCAB_CM_IDLE)
        {
            yWarning ("One motor is in idle state. Turning off control.");
            turn_off_control();
            break;
        }
    */
}

void CER_MotorControl::printStats()
{
    yInfo( "* Motor thread:\n");
    yInfo( "timeouts: %d\n", thread_timeout_counter);

    double val = 0;
    for (int i=0; i<2; i++)
    {
        if      (i==0) val=F[0];
        else if (i==1) val=F[1];
        if (board_control_modes[i]==VOCAB_CM_IDLE)
            yInfo( "F%d: IDLE\n", i);
        else
            yInfo( "F%d: %+.1f\n", i, val);
    }
}

void CER_MotorControl::close()
{
}

CER_MotorControl::~CER_MotorControl()
{
    close();
}

bool CER_MotorControl::open(ResourceFinder &_rf, Property &_options)
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
    ok = ok & control_board_driver->view(iopl);
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

CER_MotorControl::CER_MotorControl(unsigned int _period, PolyDriver* _driver) : MotorControl(_period, _driver)
{
    control_board_driver = _driver;

    thread_timeout_counter = 0;

    F.resize(2, 0.0);
    board_control_modes.resize(2, 0);

    max_linear_vel = DEFAULT_MAX_LINEAR_VEL;
    max_angular_vel = DEFAULT_MAX_ANGULAR_VEL;

    thread_period = _period;
}

void CER_MotorControl::decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    //wheel contribution calculation
    F[0] = appl_linear_speed * cos(appl_desired_direction / 180.0 * 3.14159265) + appl_angular_speed;
    F[1] = appl_linear_speed * cos(appl_desired_direction / 180.0 * 3.14159265) - appl_angular_speed;
}

void CER_MotorControl::execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    decouple(appl_linear_speed, appl_desired_direction, appl_angular_speed);
    //Use a low pass filter to obtain smooth control
    for (size_t i=0; i < F.size(); i++)
    {
        apply_motor_filter(i);
    }

    //Apply the commands
#ifdef  CONTROL_DEBUG
    yDebug (">**: %+6.6f %+6.6f **** %+6.6f %+6.6f\n",exec_linear_speed,exec_desired_direction,-F_L,-F_R);
#endif
    ivel->velocityMove(0,-F[0]);
    ivel->velocityMove(1,-F[1]);
}

void CER_MotorControl::execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    decouple(appl_linear_speed, appl_desired_direction, appl_angular_speed);
    //Use a low pass filter to obtain smooth control
    for (size_t i=0; i < F.size(); i++)
    {
        apply_motor_filter(i);
    }

    //Apply the commands
    iopl->setRefOutput(0, -F[0]);
    iopl->setRefOutput(1, -F[1]);
}

void CER_MotorControl::execute_none()
{
    iopl->setRefOutput(0,0);
    iopl->setRefOutput(1,0);
}
