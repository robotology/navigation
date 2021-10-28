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

#include "input.h"
#include "filters.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <navigation_defines.h>
#include <yarp/dev/INavigation2D.h>

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif 

YARP_LOG_COMPONENT(INPUT_HND, "navigation.baseControl.input")

void Input::printStats()
{
    yCInfo(INPUT_HND, "* Input thread:\n");
    string ss;

#if 0
    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        ss +=
        it->m_dd.close();
    }

    yCInfo(INPUT_HND, "timeouts: %d joy1: %d joy2: %d aux: %d cmd: %d\n", thread_timeout_counter, aux_timeout_counter, mov_timeout_counter);
#endif
}

void Input::close()
{
    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        if (it->m_dd)
        {
            it->m_dd->close();
            delete it->m_dd;
        }
    }
}

Input::~Input()
{
    close();
}

bool Input::open(Property &_options)
{
    ctrl_options = _options;
    string ctrl_options_string = _options.toString();
    localName    = ctrl_options.find("local").asString();

    if (!ctrl_options.check("BASECTRL_GENERAL"))
    {
        yCError(INPUT_HND) << "Missing [BASECTRL_GENERAL] section";
        return false;
    }
    yarp::os::Bottle& general_options = ctrl_options.findGroup("BASECTRL_GENERAL");
    localName = ctrl_options.find("local").asString();

    string general_options_string = general_options.toString();
    size_t input_size= general_options.find("number_of_inputs").asInt();
    if (input_size == 0)
    {
        yCError(INPUT_HND) << "Missing or invalid `number_of_inputs` param in `BASECTRL_GENERAL` section";
        return false;
    }

    for (size_t i=0; i< input_size; i++)
    {
        char buff[30];
        snprintf(buff,30,"BASECTRL_INPUTS_%d",i);
        if (!ctrl_options.check(buff))
        {
            yCError(INPUT_HND) << "Missing or invalid "<< buff << " parameters section";
            return false;
        }
        yarp::os::Bottle& input_options = ctrl_options.findGroup(buff);

        std::string nws_device_name; //should be something like: mobileBaseVelocityControl_nws_yarp
        nws_device_name = input_options.find("nws_name").asString();

        std::string nws_local;
        nws_local = input_options.find("nws_local").asString();

        inputManager anInputM;
        Property dev_options;
        dev_options.put("device", nws_device_name);
        dev_options.put("local", nws_local);
        dev_options.put("subdevice", "velocityInputHandler");
        dev_options.put("max_timeout", "0.1"); //velocityInputHandler

        anInputM.m_dd = new yarp::dev::PolyDriver();
        if (anInputM.m_dd->open(dev_options) == false)
        {
            yCError(INPUT_HND) << "Unable to open the input device `mobileBaseVelocityControl_nws_yarp`";
            return false;
        }
        else
        {
            anInputM.m_dd->view(anInputM.m_iVel);
            if (anInputM.m_dd->isValid() == false || anInputM.m_iVel == nullptr)
            {
                yCError(INPUT_HND) << "Unable to view input interface";
                return false;
            }
            else
            {
                this->m_input.push_back(anInputM);
            }
        }

    }

    return true;
}

Input::Input()
{
}

void Input::read_percent_polar(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    des_dir  = b->get(1).asFloat64();
    lin_spd  = b->get(2).asFloat64();
    ang_spd  = b->get(3).asFloat64();
    pwm_gain = b->get(4).asFloat64();
    pwm_gain = (pwm_gain<+100) ? pwm_gain : +100; 
    pwm_gain = (pwm_gain>0)    ? pwm_gain : 0;
}

void Input::read_percent_cart(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    double x_speed = b->get(1).asFloat64();
    double y_speed = b->get(2).asFloat64();
    double t_speed = b->get(3).asFloat64();
    pwm_gain       = b->get(4).asFloat64();
    des_dir        = atan2(y_speed, x_speed) * RAD2DEG;
    lin_spd        = sqrt (x_speed*x_speed+y_speed*y_speed);
    ang_spd        = t_speed;
    pwm_gain       = (pwm_gain<+100) ? pwm_gain : +100;
    pwm_gain       = (pwm_gain>0) ? pwm_gain : 0;
}

void Input::read_speed_polar(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    des_dir  = b->get(1).asFloat64();
    lin_spd  = b->get(2).asFloat64();
    ang_spd  = b->get(3).asFloat64();
    pwm_gain = b->get(4).asFloat64();
    pwm_gain = (pwm_gain<+100) ? pwm_gain : +100;
    pwm_gain = (pwm_gain>0) ? pwm_gain : 0;
}

void Input::read_speed_cart(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
     double x_speed = b->get(1).asFloat64();
     double y_speed = b->get(2).asFloat64();
     double t_speed = b->get(3).asFloat64();
     des_dir        = atan2(y_speed, x_speed) * RAD2DEG;
     lin_spd        = sqrt (x_speed*x_speed+y_speed*y_speed);
     ang_spd        = t_speed;
     pwm_gain = b->get(4).asFloat64();
}

void Input::read_inputs(double& linear_speed,double& angular_speed,double& desired_direction, double& pwm_gain)
{
#if 0
    //- - -read joysticks - - -
    for (int id=0; id<2; id++)
    {
        //port joystick
        if (port_joystick_control[id])
        {
            if (Bottle* b = port_joystick_control[id]->read(false))
            {                
                if (b->get(0).asInt32()== BASECONTROL_COMMAND_PERCENT_POLAR)
                {
                    //received a joystick command.
                    read_percent_polar(b, joy_desired_direction[id],joy_linear_speed[id],joy_angular_speed[id],joy_pwm_gain[id]);
                    joy_linear_speed[id] = (joy_linear_speed[id] > 100) ? 100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] > 100) ? 100 : joy_angular_speed[id];
                    joy_linear_speed[id] = (joy_linear_speed[id] < -100) ? -100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] < -100) ? -100 : joy_angular_speed[id];
                    joy_linear_speed[id] = joy_linear_speed[id] / 100 * linear_vel_at_100_joy;
                    joy_angular_speed[id] = joy_angular_speed[id] / 100 * angular_vel_at_100_joy;
                    wdt_old_joy_cmd = wdt_joy_cmd[id];
                    wdt_joy_cmd[id] = Time::now();

                    //Joystick commands have higher priority respect to movement commands.
                    //this make the joystick to take control for 100*20 ms
                    if (joy_pwm_gain[id]>10) joystick_received[id] = 100;
                }
                else if (b->get(0).asInt32() == BASECONTROL_COMMAND_VELOCIY_POLAR)
                {
                    //received a joystick command.
                    read_speed_polar(b, joy_desired_direction[id], joy_linear_speed[id], joy_angular_speed[id], joy_pwm_gain[id]);
                    joy_linear_speed[id] = (joy_linear_speed[id] > 100) ? 100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] > 100) ? 100 : joy_angular_speed[id];
                    joy_linear_speed[id] = (joy_linear_speed[id] < -100) ? -100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] < -100) ? -100 : joy_angular_speed[id];
                    joy_linear_speed[id] = joy_linear_speed[id] / 100 * linear_vel_at_100_joy;
                    joy_angular_speed[id] = joy_angular_speed[id] / 100 * angular_vel_at_100_joy;
                    wdt_old_joy_cmd = wdt_joy_cmd[id];
                    wdt_joy_cmd[id] = Time::now();

                    //Joystick commands have higher priority respect to movement commands.
                    //this make the joystick to take control for 100*20 ms
                    if (joy_pwm_gain[id]>10) joystick_received[id] = 100;
                }
                else if (b->get(0).asInt32() == BASECONTROL_COMMAND_VELOCIY_CARTESIAN)
                {
                    //received a joystick command.
                    read_speed_cart(b, joy_desired_direction[id], joy_linear_speed[id], joy_angular_speed[id], joy_pwm_gain[id]);
                    joy_linear_speed[id] = (joy_linear_speed[id] > 100) ? 100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] > 100) ? 100 : joy_angular_speed[id];
                    joy_linear_speed[id] = (joy_linear_speed[id] < -100) ? -100 : joy_linear_speed[id];
                    joy_angular_speed[id] = (joy_angular_speed[id] < -100) ? -100 : joy_angular_speed[id];
                    joy_linear_speed[id] = joy_linear_speed[id] / 100 * linear_vel_at_100_joy;
                    joy_angular_speed[id] = joy_angular_speed[id] / 100 * angular_vel_at_100_joy;
                    wdt_old_joy_cmd = wdt_joy_cmd[id];
                    wdt_joy_cmd[id] = Time::now();

                    //Joystick commands have higher priority respect to movement commands.
                    //this make the joystick to take control for 100*20 ms
                    if (joy_pwm_gain[id]>10) joystick_received[id] = 100;
                }
                else
                {
                    yCError(INPUT_HND) << "Invalid format received on port_joystick_control";
                }
            }
        }
        //- device joystick
        else if(iJoy[id])
        {
            read_joystick_data(&jDescr[id], iJoy[id],joy_desired_direction[id], joy_linear_speed[id], joy_angular_speed[id], joy_pwm_gain[id]);
            joy_linear_speed[id] = (joy_linear_speed[id] > 100) ? 100 : joy_linear_speed[id];
            joy_angular_speed[id] = (joy_angular_speed[id] > 100) ? 100 : joy_angular_speed[id];
            joy_linear_speed[id] = (joy_linear_speed[id] < -100) ? -100 : joy_linear_speed[id];
            joy_angular_speed[id] = (joy_angular_speed[id] < -100) ? -100 : joy_angular_speed[id];
            joy_linear_speed[id] = joy_linear_speed[id] / 100 * linear_vel_at_100_joy;
            joy_angular_speed[id] = joy_angular_speed[id] / 100 * angular_vel_at_100_joy;
            wdt_old_joy_cmd = wdt_joy_cmd[id];
            wdt_joy_cmd[id] = Time::now();

            //Joystick commands have higher priority respect to movement commands.
            //this make the joystick to take control for 100*20 ms
            if (joy_pwm_gain[id]>10) joystick_received[id] = 100;
        }
    }

    //- - - read command port - - -
    if (Bottle *b = port_movement_control.read(false))
    {
        if (b->get(0).asInt32()== BASECONTROL_COMMAND_PERCENT_POLAR)
        {
            read_percent_polar(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else if (b->get(0).asInt32()== BASECONTROL_COMMAND_VELOCIY_POLAR)
        {
            read_speed_polar(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else if (b->get(0).asInt32()== BASECONTROL_COMMAND_VELOCIY_CARTESIAN)
        {
            read_speed_cart(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else
        {
            yCError(INPUT_HND) << "Invalid format received on port_movement_control";
        }
    }

    //- - -read aux port - - -
    if (yarp::dev::MobileBaseVelocity *b = port_auxiliary_control.read(false))
    {
        aux_desired_direction = atan2(b->vel_y, b->vel_x)*RAD2DEG;
        aux_linear_speed  = sqrt((b->vel_x*b->vel_x)+ (b->vel_y * b->vel_y));
        aux_angular_speed = b->vel_theta;
        aux_pwm_gain = 100;
        auxiliary_received = 100;
        wdt_aux_cmd = Time::now();
    }
#endif

    //- - - read data - - -
    for (auto it = m_input.begin(); it!=m_input.end(); it++)
    {
        if (it->m_iVel)
        {
            double x=0;
            double y=0;
            double t=0;
            bool rec = it->m_iVel->getLastVelocityCommand(x,y,t);
            if (rec)
            {
                Bottle b;
                b.addInt32(3);
                b.addFloat64(x);
                b.addFloat64(y);
                b.addFloat64(t);
                b.addFloat64(100);
                read_speed_cart(&b, it->m_desired_direction,
                                    it->m_linear_speed,
                                    it->m_angular_speed,
                                    it->m_pwm_gain);
                it->m_old_wdt = it->m_wdt;
                it->m_wdt = Time::now();
                it->data_received=100;
            }
        }
    }

    //- - - priority test - - -
    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        if (it->data_received > 0)
        {
            desired_direction = it->m_desired_direction;
            linear_speed = it->m_linear_speed;
            angular_speed = it->m_angular_speed;
            pwm_gain = it->m_pwm_gain;
            break;
        }
    }

    //watchdog on received commands
    static double wdt_old=Time::now();
    double wdt=Time::now();
    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        if (wdt - it->m_wdt > 0.200)
        {
            it->m_desired_direction = 0;
            it->m_linear_speed = 0;
            it->m_angular_speed = 0;
            it->m_pwm_gain = 0;
            it->m_timeout_counter++;
        }
    }

    if (wdt-wdt_old > 0.040) { thread_timeout_counter++;  }
    wdt_old=wdt;

    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        if (it->data_received > 0)
        {
            it->data_received--;
        }
    }
}
