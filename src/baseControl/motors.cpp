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
    if (enable_ROS && enable_ROS_OUTPUT_GROUP)
    {
        rosPublisherPort_cmd_twist.interrupt();
        rosPublisherPort_cmd_twist.close();
    }

    port_status.interrupt();
    port_status.close();
}

MotorControl::~MotorControl()
{
    close();
}

void MotorControl::execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed)
{
    if (enable_ROS_OUTPUT_GROUP)
    {
        geometry_msgs_Twist &twist = rosPublisherPort_cmd_twist.prepare();

        twist.linear.x = appl_linear_speed * cos(appl_desired_direction*0.0174532925);
        twist.linear.y = appl_linear_speed * sin(appl_desired_direction*0.0174532925);
        twist.linear.z = 0;

        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = appl_angular_speed*0.0174532925;

        rosPublisherPort_cmd_twist.write();
    }
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
    else if (motors_filter_enabled == 10)
    {
        F[i] = control_filters::lp_filter_0_5Hz(F[i], i);
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

    if (ctrl_options.check("GENERAL"))
    {
        yarp::os::Bottle g_group = ctrl_options.findGroup("GENERAL");
        enable_ROS = (g_group.find("use_ROS").asBool() == true);
        if (enable_ROS) yInfo() << "ROS enabled";
        else
            yInfo() << "ROS not enabled";
    }
    else
    {
        yError() << "Missing [GENERAL] section";
        return false;
    }

    if (!ctrl_options.check("MOTORS"))
    {
        yError() << "Missing [MOTORS] section";
        return false;
    }
    yarp::os::Bottle& motors_options = ctrl_options.findGroup("MOTORS");

    if (motors_options.check("motors_filter_enabled") == false)
    {
        yError() << "Missing param motors_filter_enabled";
        return false;
    }
    
    if (motors_options.check("max_motor_pwm") == false)
    {
        yError() << "Missing param max_motor_pwm";
        return false;
    }

    if (motors_options.check("max_motor_vel") == false)
    {
        yError() << "Missing param max_motor_vel";
        return false;
    }

    motors_filter_enabled = motors_options.check("motors_filter_enabled", Value(4), "motors filter frequency (1/2/4/8Hz, 0 = disabled)").asInt();
    max_motor_pwm = motors_options.check("max_motor_pwm", Value(0), "max_motor_pwm").asDouble();
    max_motor_vel = motors_options.check("max_motor_vel", Value(0), "max_motor_vel").asDouble();

    localName = ctrl_options.find("local").asString();

    if (enable_ROS)
    {
        if (ctrl_options.check("ROS_OUTPUT"))
        {
            yarp::os::Bottle rout_group = ctrl_options.findGroup("ROS_OUTPUT");
            if (rout_group.check("topic_name") == false)  { yError() << "Missing topic_name parameter"; return false; }
            rosTopicName_cmd_twist = rout_group.find("topic_name").asString();
            enable_ROS_OUTPUT_GROUP = true;
        }
        else
        {
            enable_ROS_OUTPUT_GROUP = false;
        }

        if (!rosPublisherPort_cmd_twist.topic(rosTopicName_cmd_twist))
        {
            yError() << " opening " << rosTopicName_cmd_twist << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }
        yInfo () << "ROS_OUTPUT param found. Enabling topic "<<rosTopicName_cmd_twist;
    }

    bool ret = port_status.open((localName+"/motors_status:o").c_str());
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    return true;
}

MotorControl::MotorControl(unsigned int _period, PolyDriver* _driver)
{
    control_board_driver = _driver;
    motors_num=0;
    thread_timeout_counter = 0;

    max_motor_vel = 0;
    max_motor_pwm = 0;

    thread_period = _period;
}

void MotorControl::printStats()
{
    yInfo( "* Motor thread:\n");
    yInfo( "timeouts: %d\n", thread_timeout_counter);

    double val = 0;
    for (int i=0; i<motors_num; i++)
    {
        val=F[i];
        if (board_control_modes[i]==VOCAB_CM_IDLE)
            yInfo( "F%d: IDLE\n", i);
        else
            yInfo( "F%d: %+.1f\n", i, val);
    }

    if (port_status.getOutputCount()>0)
    {
        //port_status.setEnvelope(timeStamp);
        Bottle &t = port_status.prepare();
        t.clear();
        for (int i=0; i<motors_num; i++)
        {
            string s = yarp::os::Vocab::decode(board_control_modes[i]);
            t.addString(s);
        }
        port_status.write();
    }
}

bool MotorControl::set_control_openloop()
{
    yInfo ("Setting openloop mode");
    for (int i=0; i<motors_num; i++)
    {
        icmd->setControlMode(i, VOCAB_CM_PWM);
        ipwm->setRefDutyCycle(i, 0);
    }
    return true;
}

bool MotorControl::set_control_velocity()
{
    yInfo ("Setting velocity mode");
    for (int i=0; i<motors_num; i++)
    {
        icmd->setControlMode(i, VOCAB_CM_VELOCITY);
        ivel->setRefAcceleration(i, 1000000);
        ivel->velocityMove(i, 0);
    }
    return true;
}

bool MotorControl::set_control_idle()
{
    yInfo ("Setting ilde mode");
    for (int i=0; i<motors_num; i++)
    {
        icmd->setControlMode(i, VOCAB_CM_IDLE);
    }
    yInfo("Motors now off");
    return true;
}

bool MotorControl::check_motors_on()
{
    int c0(0),c1(0);
    yarp::os::Time::delay(0.05);
    
    bool status_not_idle=true;
    for (int i=0; i<motors_num; i++)
    {
        icmd->getControlMode(i, &board_control_modes[i]);
        status_not_idle &= (board_control_modes[i]!=VOCAB_CM_IDLE);
    }

    if (status_not_idle)
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

void MotorControl::updateControlMode()
{
    board_control_modes_last = board_control_modes;
    
    for (int i = 0; i < motors_num; i++)
    {
        icmd->getControlMode(i, &board_control_modes[i]);
        if (board_control_modes[i] == VOCAB_CM_HW_FAULT && board_control_modes_last[i] != VOCAB_CM_HW_FAULT)
        {
            yWarning("One motor is in fault status. Turning off control.");
            set_control_idle();
            break;
        }
    }
}