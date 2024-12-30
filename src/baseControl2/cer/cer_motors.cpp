/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "cer_motors.h"
#include "../filters.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <limits>

YARP_LOG_COMPONENT(CER_MOT, "navigation.baseControl.cerMotorControl")

void CER_MotorControl::close()
{
}

CER_MotorControl::~CER_MotorControl()
{
    close();
}

bool CER_MotorControl::open(const Property &_options)
{
    ctrl_options = _options;
    localName = ctrl_options.find("local").asString();

    //the base class open
    if (!MotorControl::open(_options))
    {
        yCError(CER_MOT) << "Error in MotorControl::open()"; return false;
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
        yCError(CER_MOT,"One or more devices has not been viewed, returning\n");
        return false;
    }

    if (!ctrl_options.check("BASECTRL_GENERAL"))
    {
        yCError(CER_MOT) << "Missing [BASECTRL_GENERAL] section";
        return false;
    }
    yarp::os::Bottle& general_options = ctrl_options.findGroup("BASECTRL_GENERAL");

    //get robot geometry
    Bottle geometry_group = ctrl_options.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError(CER_MOT,"Unable to find ROBOT_GEOMETRY group!");
        return false;
    }
    if (!geometry_group.check("geom_r"))
    {
        yCError(CER_MOT,"Missing param geom_r in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("geom_L"))
    {
        yCError(CER_MOT,"Missing param geom_L in [ROBOT_GEOMETRY] group");
        return false;
    }
    geom_r = geometry_group.find("geom_r").asFloat64();
    geom_L = geometry_group.find("geom_L").asFloat64();

    localName = ctrl_options.find("local").asString();

    return true;
}

CER_MotorControl::CER_MotorControl(PolyDriver* _driver) : MotorControl(_driver)
{
    control_board_driver = _driver;
    motors_num=2;
    thread_timeout_counter = 0;

    F.resize(motors_num, 0.0);
    board_control_modes.resize(motors_num, 0);
    board_control_modes_last.resize(motors_num, 0);

    geom_r = 0;
    geom_L = 0;
}

void CER_MotorControl::decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    //wheel contribution calculation
    F[0] = appl_linear_speed * cos(appl_desired_direction * DEG2RAD) - appl_angular_speed;
    F[1] = appl_linear_speed * cos(appl_desired_direction * DEG2RAD) + appl_angular_speed;
}

void CER_MotorControl::execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    MotorControl::execute_speed(appl_linear_speed, appl_desired_direction, appl_angular_speed);

    double appl_angular_speed_to_wheels = appl_angular_speed * this->get_vang_coeff();
    double appl_linear_speed_to_wheels = appl_linear_speed * this->get_vlin_coeff();
    decouple(appl_linear_speed_to_wheels, appl_desired_direction, appl_angular_speed_to_wheels);

    //yCDebug() << appl_linear_speed << appl_linear_speed_to_wheels;
    //Use a low pass filter to obtain smooth control
    for (size_t i=0; i < F.size(); i++)
    {
        apply_motor_filter(i);
    }

    //Apply the commands
    ivel->velocityMove(0,F[0]);
    ivel->velocityMove(1,F[1]);
    //yCDebug() << F[0] << F[1];
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
    ipwm->setRefDutyCycle(0, F[0]);
    ipwm->setRefDutyCycle(1, F[1]);
}

void CER_MotorControl::execute_none()
{
    ipwm->setRefDutyCycle(0, 0);
    ipwm->setRefDutyCycle(1, 0);
}

double CER_MotorControl::get_vlin_coeff()
{
    return 1 / geom_r * RAD2DEG;
}

double CER_MotorControl::get_vang_coeff()
{
      return geom_L / (2 * geom_r);
}
