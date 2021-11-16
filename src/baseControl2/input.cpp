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
#include <yarp/dev/IWrapper.h>
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
    string ss = "timeouts:";

    ss = ss+ "thread:" + std::to_string(thread_timeout_counter);

    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        ss = ss + " (" + it->m_name;
        ss = ss + ":" + std::to_string(it->m_timeout_counter) + ")";
    }
    yCInfo(INPUT_HND) << ss;
}

void Input::close()
{
    for (auto it = m_input.begin(); it != m_input.end(); it++)
    {
        if (it->m_nws_dd)
        {
            it->m_nws_dd->close();
            delete it->m_nws_dd;
        }
        if (it->m_inputmanager_dd)
        {
            it->m_inputmanager_dd->close();
            delete it->m_inputmanager_dd;
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

    //read section [BASECTRL_INPUTS_XXX]
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

        //read param nws_name
        std::string nws_device_name; //should be something like: mobileBaseVelocityControl_nws_yarp
        nws_device_name = input_options.find("nws_name").asString();
        if (nws_device_name.empty()) {
            yCError(INPUT_HND) << "Missing or invalid `nws_device_name` parameter";
            return false;
        }

        //read param input_name
        std::string input_name;
        input_name = input_options.find("input_name").asString();
        if (input_name.empty()) { yCError(INPUT_HND) << "Missing or invalid `input_name` parameter";
            return false;
        }

        //read param max_timeout
        double max_timeout=0.1;
        if (input_options.check("max_timeout"))
        {
            max_timeout = input_options.find("max_timeout").asFloat64();
        }

        //open the nws
        inputManager anInputM;
        Property nws_options;
        nws_options.fromString(input_options.toString());
        nws_options.put("device", nws_device_name);
        anInputM.m_nws_dd = new yarp::dev::PolyDriver();
        if (anInputM.m_nws_dd->open(nws_options) == false)
        {
            yCError(INPUT_HND) << "Unable to open the input device `mobileBaseVelocityControl_nws_yarp`";
            return false;
        }
        yarp::dev::IWrapper* iwrap = nullptr;
        anInputM.m_nws_dd->view(iwrap);
        if (anInputM.m_nws_dd->isValid() == false || iwrap == nullptr)
        {
            yCError(INPUT_HND) << "Unable to view IWrapper interface";
            return false;
        }

        //open the input manager
        Property inputManager_options;
        inputManager_options.put("device", "velocityInputHandler");
        inputManager_options.put("max_timeout", max_timeout);
        anInputM.m_inputmanager_dd = new yarp::dev::PolyDriver();
        if (anInputM.m_inputmanager_dd->open(inputManager_options) == false)
        {
            yCError(INPUT_HND) << "Unable to open the input device `velocityInputHandler`";
            return false;
        }
        anInputM.m_inputmanager_dd->view(anInputM.m_iVel);
        if (anInputM.m_inputmanager_dd->isValid() == false || anInputM.m_iVel == nullptr)
        {
            yCError(INPUT_HND) << "Unable to view input interface";
            return false;
        }

        //attach the two devices
        if (iwrap->attach(anInputM.m_inputmanager_dd) == false)
        {
            yCError(INPUT_HND) << "Cannot attach the nws to its input device";
            return false;
        }

        //add the new nws+inputdevice to the list of inputs
        anInputM.m_name = input_name;
        this->m_input.push_back(anInputM);
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
                it->data_received=100;
            }
            else
            {
                it->m_desired_direction=0;
                it->m_linear_speed=0;
                it->m_angular_speed=0;
                it->m_pwm_gain=0;
                //counting the TOTAL of timeouts. The counter NEVER resets.
                it->m_timeout_counter++;
            }

            //countdown
            if (it->data_received > 0)
            {
                it->data_received--;
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

    //thread watchdog
    static double wdt_old=Time::now();
    double wdt=Time::now();
    if (wdt-wdt_old > 0.040) { thread_timeout_counter++;  }
    wdt_old=wdt;
}
