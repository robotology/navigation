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

#include "ikart_motors.h"
#include "../filters.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

bool iKart_MotorControl::set_control_openloop()
{
    yInfo ("Setting openloop mode");
    icmd->setOpenLoopMode(0);
    icmd->setOpenLoopMode(1);
    iopl->setRefOutput(0,0);
    iopl->setRefOutput(1,0);
    return true;
}

bool iKart_MotorControl::set_control_velocity()
{
    yInfo ("Setting velocity mode");
    icmd->setVelocityMode(0);
    icmd->setVelocityMode(1);
    ivel->velocityMove(0,0);
    ivel->velocityMove(1,0);
    return true;
}

bool iKart_MotorControl::set_control_idle()
{
    yInfo ("Setting ilde mode");
    icmd->setControlMode(0, VOCAB_CM_IDLE);
    icmd->setControlMode(1, VOCAB_CM_IDLE);
    icmd->setControlMode(2, VOCAB_CM_IDLE);
    icmd->setControlMode(3, VOCAB_CM_IDLE);
    yInfo("Motors now off");
    return true;
}

bool iKart_MotorControl::check_motors_on()
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

void iKart_MotorControl::updateControlMode()
{
    board_control_modes_last = board_control_modes;
    icmd->getControlMode(0, &board_control_modes[0]);
    icmd->getControlMode(1, &board_control_modes[1]);
    icmd->getControlMode(2, &board_control_modes[2]);

    for (int i = 0; i < 3; i++)
    {
        if (board_control_modes[i] == VOCAB_CM_HW_FAULT && board_control_modes_last[i] != VOCAB_CM_HW_FAULT)
        {
            yWarning("One motor is in fault status. Turning off control.");
            set_control_idle();
            break;
        }
    }
}

void iKart_MotorControl::printStats()
{
    yInfo( "* Motor thread:\n");
    yInfo( "timeouts: %d\n", thread_timeout_counter);

    double val = 0;
    for (int i=0; i<3; i++)
    {
        if      (i==0) val = F[0];
        else if (i==1) val = F[1];
        else if (i==2) val = F[2];
        if (board_control_modes[i]==VOCAB_CM_IDLE)
            yInfo( "F%d: IDLE\n", i);
        else
            yInfo( "F%d: %+.1f\n", i, val);
    }
}

void iKart_MotorControl::close()
{
}

iKart_MotorControl::~iKart_MotorControl()
{
    close();
}

bool iKart_MotorControl::open(ResourceFinder &_rf, Property &_options)
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

    localName = ctrl_options.find("local").asString();

    return true;
}

iKart_MotorControl::iKart_MotorControl(unsigned int _period, PolyDriver* _driver) : MotorControl(_period, _driver)
{
    control_board_driver = _driver;

    thread_timeout_counter = 0;

    F.resize(3,0.0);
    board_control_modes.resize(3, 0);
    board_control_modes_last.resize(3, 0);

    thread_period = _period;
}

void iKart_MotorControl::decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    //wheel contribution calculation
    double wheels_off = 0;

    F[0] = appl_linear_speed * cos((150.0 - appl_desired_direction + wheels_off) / 180.0 * 3.14159265) + appl_angular_speed;
    F[1] = appl_linear_speed * cos((030.0 - appl_desired_direction + wheels_off) / 180.0 * 3.14159265) + appl_angular_speed;
    F[2] = appl_linear_speed * cos((270.0 - appl_desired_direction + wheels_off) / 180.0 * 3.14159265) + appl_angular_speed;
}

void iKart_MotorControl::execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    decouple(appl_linear_speed, appl_desired_direction,appl_angular_speed);
    //Use a low pass filter to obtain smooth control
    //Use a low pass filter to obtain smooth control
    for (size_t i=0; i < F.size(); i++)
    {
        apply_motor_filter(i);
    }

    //Apply the commands
    ivel->velocityMove(0, -F[0]);
    ivel->velocityMove(1, -F[1]);
    ivel->velocityMove(2, -F[2]);
}

void iKart_MotorControl::execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    decouple(appl_linear_speed, appl_desired_direction,appl_angular_speed);

    //Use a low pass filter to obtain smooth control
    //Use a low pass filter to obtain smooth control
    for (size_t i=0; i < F.size(); i++)
    {
        apply_motor_filter(i);
    }

    //Apply the commands
    iopl->setRefOutput(0, -F[0]);
    iopl->setRefOutput(1, -F[1]);
    iopl->setRefOutput(1, -F[2]);
}

void iKart_MotorControl::execute_none()
{
    iopl->setRefOutput(0,0);
    iopl->setRefOutput(1,0);
    iopl->setRefOutput(2,0);
}
