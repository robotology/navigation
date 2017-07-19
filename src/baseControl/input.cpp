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

#define _USE_MATH_DEFINES
#include <math.h>

void Input::printStats()
{
    yInfo( "* Input thread:\n");
    yInfo( "timeouts: %d joy: %d cmd %d\n", thread_timeout_counter, joy_timeout_counter, mov_timeout_counter);

    if (joystick_received>0) 
        yInfo( "Under joystick control (%d)\n", joystick_received);
}

void Input::close()
{
    port_movement_control.interrupt();
    port_movement_control.close();
    port_auxiliary_control.interrupt();
    port_auxiliary_control.close();
    port_joystick_control.interrupt();
    port_joystick_control.close();
}

Input::~Input()
{
    close();
}

bool Input::open(ResourceFinder &_rf, Property &_options)
{
    ctrl_options = _options;
    localName    = ctrl_options.find("local").asString();

    // open control input ports
    port_movement_control.open((localName+"/control:i").c_str());
    port_auxiliary_control.open((localName+"/aux_control:i").c_str());

    if (!ctrl_options.check("GENERAL"))
    {
        yError() << "Missing [GENERAL] section";
        return false;
    }
    yarp::os::Bottle& general_options = ctrl_options.findGroup("GENERAL");
    yarp::os::Bottle& joystick_options = ctrl_options.findGroup("JOYSTICK");

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
    useRos = general_options.check("use_ROS", Value(false), "enable Ros Comunication").asBool();

    if (useRos)
    {
        if (ctrl_options.check("ROS_INPUT"))
        {
            yarp::os::Bottle rin_group = ctrl_options.findGroup("ROS_INPUT");
            if (rin_group.check("topic_name") == false)  { yError() << "Missing topic_name parameter"; return false; }
            rosTopicName_twist = rin_group.find("topic_name").asString();
            enable_ROS_INPUT_GROUP = true;
        }
        else
        {
            enable_ROS_INPUT_GROUP = false;
        }
        
        if (!rosSubscriberPort_twist.topic(rosTopicName_twist))
        {
            yError() << " opening " << rosTopicName_twist << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }
    }
    

    double tmp = 0;
    tmp = (joystick_options.check("linear_vel_at_full_control", Value(0), "linear velocity at 100% of the joystick control [m/s]")).asDouble();
    if (tmp>0) linear_vel_at_100_joy = tmp;
    tmp = (joystick_options.check("angular_vel_at_full_control", Value(0), "angular velocity at 100% of the joystick control [deg/s]")).asDouble();
    if (tmp>0) angular_vel_at_100_joy = tmp;

    localName = ctrl_options.find("local").asString();

    //Joystick
    if (general_options.check("joypad_input_group")==false)
    {
        yError() << "missing parameter joypad_input_group in [GENERAL] group";
        return false;
    }

    string joypad_group_name = general_options.find("joypad_input_group").toString();
    Bottle& joypad_group = ctrl_options.findGroup(joypad_group_name);
    if (joypad_group.isNull())
    {
        yError() << "Unable to find joypad section" << joypad_group_name;
        return false;
    }

    if (joypad_group.check("JoystickPort"))
    {
        string joystick_port_name = joypad_group.find("JoystickPort").asString();
        port_joystick_control.open(joystick_port_name.c_str());
    }
    else if (joypad_group.check("JoypadDevice"))
    {
        Input::JoyDescription joydesc;
        Value joydevicename = joypad_group.find("JoypadDevice");
        if (!joydevicename.isString())
        {
            yError() << "baseControl: JoypadDevice param is not a string";
            return false;
        }
        Value joylocal = joypad_group.find("local");
        if (!joylocal.isString())
        {
            yError() << "baseControl: JoypadDevice param is not a string";
            return false;
        }
        Value joyremote = joypad_group.find("remote");
        if (!joyremote.isString())
        {
            yError() << "baseControl: JoypadDevice param is not a string";
            return false;
        }

        typedef Input::InputDescription::InputType inputType;
        vector<tuple<string, unsigned int*, inputType*, float*> > paramlist;
        paramlist.push_back(make_tuple("x",    &joydesc.xAxis.Id, &joydesc.xAxis.type, &joydesc.xAxis.Factor));
        paramlist.push_back(make_tuple("y",    &joydesc.yAxis.Id, &joydesc.yAxis.type, &joydesc.yAxis.Factor));
        paramlist.push_back(make_tuple("t",    &joydesc.tAxis.Id, &joydesc.tAxis.type, &joydesc.tAxis.Factor));
        paramlist.push_back(make_tuple("gain", &joydesc.gain.Id,  &joydesc.gain.type,  &joydesc.gain.Factor));

        for(auto p : paramlist)
        {
            bool                   idFound(false), facFound(false);
            string                 idPar;
            string                 factorPar;
            string                 Error("Axis/Button");
            inputType              type;
            map<string, inputType> str2types{make_pair("Axis",   Input::InputDescription::AXIS),
                                             make_pair("Button", Input::InputDescription::BUTTON)};
            for(auto t : str2types)
            {
                idPar     = std::get<0>(p)+t.first+"_id";
                factorPar = std::get<0>(p)+t.first+"_factor";
                if(joypad_group.check(idPar) && joypad_group.find(idPar).isInt())
                {
                    idFound = true;
                    if(joypad_group.check(factorPar) && joypad_group.find(factorPar).isDouble())
                    {
                        facFound = true;
                        type = str2types[t.first];
                        break;
                    }
                }
            }

            if(!idFound)
            {
                yError() << "baseControl: param" << std::get<0>(p)+Error+"_id" << "not found or not a int in configuration file";
                return false;
            }

            if(!facFound)
            {
                yError() << "baseControl: param" << std::get<0>(p)+Error+"_factor" << "not found or not a int in configuration file";
                return false;
            }

            if(type == Input::InputDescription::BUTTON &&
                    (std::get<0>(p) == "x" ||
                     std::get<0>(p) == "y" ||
                     std::get<0>(p) == "t"))
            {
                yError() << "at the moment xAxis, yAxis and tAxis cannot be mapped to buttons";
                return false;
            }

            *std::get<1>(p) = joypad_group.find(idPar).asInt();
            *std::get<2>(p) = type;
            *std::get<3>(p) = joypad_group.find(factorPar).asDouble();
        }

        yInfo() << "opening" << joydevicename.asString() << "device";
        Property joycfg;
        joycfg.put("device", joydevicename.asString());
        joycfg.put("local", joylocal.toString());
        joycfg.put("remote",joyremote.toString());

        if (!joyPolyDriver.open(joycfg))
        {
            yError() << "baseControl: could not open the joypad device";
            return false;
        }

        joyPolyDriver.view(joypad);
        if (!joypad)
        {
            yError() << "joypad Device must implement the IJoypadController interface";
            return false;
        }
        unsigned int count;
        joypad->getAxisCount(count);
        if(count < 4)
        {
            yError() << "joypad must have at least 3 axes";
            return false;
        }
    }
    else
    {
        yError("Unable to configure joystick: You need either the JoystickPort or the JoypadDevice option");
        return false;
    }

    return true;
}

Input::Input(unsigned int _period, PolyDriver* _driver)
{
    useRos                 = false;
    control_board_driver   = _driver;

    thread_timeout_counter = 0;

    command_received       = 0;
    rosInput_received      = 0;
    joystick_received      = 0;
    auxiliary_received     = 0;
                           
    mov_timeout_counter    = 0;
    joy_timeout_counter    = 0;
    aux_timeout_counter    = 0;
    ros_timeout_counter    = 0;

    joy_linear_speed       = 0;
    joy_angular_speed      = 0;
    joy_desired_direction  = 0;
    joy_pwm_gain           = 0;

    cmd_linear_speed       = 0;
    cmd_angular_speed      = 0;
    cmd_desired_direction  = 0;
    cmd_pwm_gain           = 0;

    aux_linear_speed       = 0;
    aux_angular_speed      = 0;
    aux_desired_direction  = 0;
    aux_pwm_gain           = 0;
    
    ros_linear_speed       = 0;
    ros_angular_speed      = 0;
    ros_desired_direction  = 0;
    ros_pwm_gain           = 0;

    linear_vel_at_100_joy  = 0;
    angular_vel_at_100_joy = 0;

    thread_period          = _period;
    joypad                 = 0;
}

void Input::read_percent_polar(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    des_dir  = b->get(1).asDouble();
    lin_spd  = b->get(2).asDouble();
    ang_spd  = b->get(3).asDouble();
    pwm_gain = b->get(4).asDouble();
    pwm_gain = (pwm_gain<+100) ? pwm_gain : +100; 
    pwm_gain = (pwm_gain>0)    ? pwm_gain : 0;
}

void Input::read_percent_cart(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    double x_speed = b->get(1).asDouble();
    double y_speed = b->get(2).asDouble();
    double t_speed = b->get(3).asDouble();
    pwm_gain       = b->get(4).asDouble();
    des_dir        = atan2(y_speed, x_speed) * 180.0 / M_PI;
    lin_spd        = sqrt (x_speed*x_speed+y_speed*y_speed);
    ang_spd        = t_speed;
    pwm_gain       = (pwm_gain<+100) ? pwm_gain : +100;
    pwm_gain       = (pwm_gain>0) ? pwm_gain : 0;
}

void Input::read_speed_polar(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    des_dir  = b->get(1).asDouble();
    lin_spd  = b->get(2).asDouble();
    ang_spd  = b->get(3).asDouble();
    pwm_gain = b->get(4).asDouble();
    pwm_gain = (pwm_gain<+100) ? pwm_gain : +100;
    pwm_gain = (pwm_gain>0) ? pwm_gain : 0;
}

void Input::read_speed_cart(const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
     double x_speed = b->get(1).asDouble();
     double y_speed = b->get(2).asDouble();
     double t_speed = b->get(3).asDouble();
     des_dir        = atan2(y_speed, x_speed) * 180.0 / M_PI;
     lin_spd        = sqrt (x_speed*x_speed+y_speed*y_speed);
     ang_spd        = t_speed;
     pwm_gain = b->get(4).asDouble();
}

void Input::read_joystick_data(double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain)
{
    //received a joystick command.
    double x_speed;
    double y_speed;
    joypad->getAxis(jDescr.xAxis.Id, x_speed);
    joypad->getAxis(jDescr.yAxis.Id, y_speed);
    joypad->getAxis(jDescr.tAxis.Id, joy_angular_speed);

    if(jDescr.gain.type == InputDescription::AXIS)
    {
        joypad->getAxis(jDescr.gain.Id, joy_pwm_gain);
    }
    else
    {
        float r;
        joypad->getButton(jDescr.gain.Id, r);
        joy_pwm_gain = r;
    }

    x_speed           *= jDescr.xAxis.Factor;
    y_speed           *= jDescr.yAxis.Factor;
    joy_angular_speed *= jDescr.tAxis.Factor;
    joy_pwm_gain      *= jDescr.gain.Factor;

    des_dir  = atan2(y_speed, x_speed) * 180.0 / M_PI;
    lin_spd  = sqrt (x_speed*x_speed+y_speed*y_speed);
    pwm_gain = (pwm_gain<+100) ? pwm_gain : +100;
    pwm_gain = (pwm_gain>0) ? pwm_gain : 0;
}

void Input::read_inputs(double *linear_speed,double *angular_speed,double *desired_direction, double *pwm_gain)
{
    static double wdt_old_mov_cmd =Time::now();
    static double wdt_old_ros_cmd =Time::now();
    static double wdt_old_joy_cmd =Time::now();
    static double wdt_old_aux_cmd =Time::now();
    static double wdt_mov_cmd     =Time::now();
    static double wdt_ros_cmd     =Time::now();
    static double wdt_joy_cmd     =Time::now();
    static double wdt_aux_cmd     =Time::now();

    if(joypad)
    {
        read_joystick_data(joy_desired_direction, joy_linear_speed, joy_angular_speed, joy_pwm_gain);
        joy_linear_speed = (joy_linear_speed > 100) ? 100 : joy_linear_speed;
        joy_angular_speed = (joy_angular_speed > 100) ? 100 : joy_angular_speed;
        joy_linear_speed = (joy_linear_speed < -100) ? -100 : joy_linear_speed;
        joy_angular_speed = (joy_angular_speed < -100) ? -100 : joy_angular_speed;
        joy_linear_speed = joy_linear_speed / 100 * linear_vel_at_100_joy;
        joy_angular_speed = joy_angular_speed / 100 * angular_vel_at_100_joy;
        wdt_old_joy_cmd = wdt_joy_cmd;
        wdt_joy_cmd = Time::now();

        //Joystick commands have higher priorty respect to movement commands.
        //this make the joystick to take control for 100*20 ms
        if (joy_pwm_gain>10) joystick_received = 100;
    }
    else if(Bottle* b = port_joystick_control.read(false))
    {                
        if (b->get(0).asInt()==1)
        {
            //received a joystick command.
            read_percent_polar(b, joy_desired_direction,joy_linear_speed,joy_angular_speed,joy_pwm_gain);
            joy_linear_speed = (joy_linear_speed > 100) ? 100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed > 100) ? 100 : joy_angular_speed;
            joy_linear_speed = (joy_linear_speed < -100) ? -100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed < -100) ? -100 : joy_angular_speed;
            joy_linear_speed = joy_linear_speed / 100 * linear_vel_at_100_joy;
            joy_angular_speed = joy_angular_speed / 100 * angular_vel_at_100_joy;
            wdt_old_joy_cmd = wdt_joy_cmd;
            wdt_joy_cmd = Time::now();

            //Joystick commands have higher priorty respect to movement commands.
            //this make the joystick to take control for 100*20 ms
            if (joy_pwm_gain>10) joystick_received = 100;
        }
        else if (b->get(0).asInt() == 2)
        {
            //received a joystick command.
            read_speed_polar(b, joy_desired_direction, joy_linear_speed, joy_angular_speed, joy_pwm_gain);
            joy_linear_speed = (joy_linear_speed > 100) ? 100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed > 100) ? 100 : joy_angular_speed;
            joy_linear_speed = (joy_linear_speed < -100) ? -100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed < -100) ? -100 : joy_angular_speed;
            joy_linear_speed = joy_linear_speed / 100 * linear_vel_at_100_joy;
            joy_angular_speed = joy_angular_speed / 100 * angular_vel_at_100_joy;
            wdt_old_joy_cmd = wdt_joy_cmd;
            wdt_joy_cmd = Time::now();

            //Joystick commands have higher priorty respect to movement commands.
            //this make the joystick to take control for 100*20 ms
            if (joy_pwm_gain>10) joystick_received = 100;
        }
        else if (b->get(0).asInt() == 3)
        {
            //received a joystick command.
            read_speed_cart(b, joy_desired_direction, joy_linear_speed, joy_angular_speed, joy_pwm_gain);
            joy_linear_speed = (joy_linear_speed > 100) ? 100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed > 100) ? 100 : joy_angular_speed;
            joy_linear_speed = (joy_linear_speed < -100) ? -100 : joy_linear_speed;
            joy_angular_speed = (joy_angular_speed < -100) ? -100 : joy_angular_speed;
            joy_linear_speed = joy_linear_speed / 100 * linear_vel_at_100_joy;
            joy_angular_speed = joy_angular_speed / 100 * angular_vel_at_100_joy;
            wdt_old_joy_cmd = wdt_joy_cmd;
            wdt_joy_cmd = Time::now();

            //Joystick commands have higher priorty respect to movement commands.
            //this make the joystick to take control for 100*20 ms
            if (joy_pwm_gain>10) joystick_received = 100;
        }
        else
        {
            yError() << "Invalid format received on port_joystick_control";
        }
    }
    if (Bottle *b = port_movement_control.read(false))
    {
        if (b->get(0).asInt()==1)
        {
            read_percent_polar(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else if (b->get(0).asInt()==2)
        {
            read_speed_polar(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else if (b->get(0).asInt()==3)
        {
            read_speed_cart(b, cmd_desired_direction,cmd_linear_speed,cmd_angular_speed,cmd_pwm_gain);
            wdt_old_mov_cmd = wdt_mov_cmd;
            wdt_mov_cmd = Time::now();
            command_received = 100;
        }
        else
        {
            yError() << "Invalid format received on port_movement_control";
        }
    }
    if (Bottle *b = port_auxiliary_control.read(false))
    {
        if (b->get(0).asInt()==1)
        {
            read_percent_polar(b, aux_desired_direction,aux_linear_speed,aux_angular_speed,aux_pwm_gain);
            wdt_old_aux_cmd = wdt_aux_cmd;
            wdt_aux_cmd = Time::now();
            auxiliary_received = 100;
        }
        else if (b->get(0).asInt()==2)
        {
            read_speed_polar(b, aux_desired_direction,aux_linear_speed,aux_angular_speed,aux_pwm_gain);
            wdt_old_aux_cmd = wdt_aux_cmd;
            wdt_aux_cmd = Time::now();
            auxiliary_received = 100;
        }
        else if (b->get(0).asInt()==3)
        {
            read_speed_cart(b, aux_desired_direction,aux_linear_speed,aux_angular_speed,aux_pwm_gain);
            wdt_old_aux_cmd = wdt_aux_cmd;
            wdt_aux_cmd = Time::now();
            auxiliary_received = 100;
        }
        else
        {
            yError() << "Invalid format received on port_auxiliary_control";
        }
    }
    
    if (geometry_msgs_Twist* rosTwist = rosSubscriberPort_twist.read(false))
    {
        Bottle b;
        b.addInt(3);
        b.addDouble(rosTwist->linear.x);
        b.addDouble(rosTwist->linear.y);
        b.addDouble(rosTwist->angular.z * 180 / M_PI);
        read_speed_cart(&b, ros_desired_direction, ros_linear_speed, ros_angular_speed, ros_pwm_gain);
        wdt_old_ros_cmd   = wdt_ros_cmd;
        wdt_ros_cmd       = Time::now();
        rosInput_received = 100;
    }

    //priority test 
    if (joystick_received>0)
    {
        *desired_direction  = joy_desired_direction;
        *linear_speed       = joy_linear_speed;
        *angular_speed      = joy_angular_speed;
        *pwm_gain           = joy_pwm_gain;
    }
    else if (auxiliary_received>0)
    {
        *desired_direction  = aux_desired_direction;
        *linear_speed       = aux_linear_speed;
        *angular_speed      = aux_angular_speed;
        *pwm_gain           = aux_pwm_gain;
    }
    else if (rosInput_received>0)
    {
        *desired_direction  = ros_desired_direction;
        *linear_speed       = ros_linear_speed;
        *angular_speed      = ros_angular_speed;
        *pwm_gain           = ros_pwm_gain;
    }
    else //if (command_received>0)
    {
        *desired_direction  = cmd_desired_direction;
        *linear_speed       = cmd_linear_speed;
        *angular_speed      = cmd_angular_speed;
        *pwm_gain           = cmd_pwm_gain;
    }

    //watchdog on received commands
    static double wdt_old=Time::now();
    double wdt=Time::now();
    //yDebug("period: %f\n", wdt-wdt_old);
    if (wdt-wdt_mov_cmd > 0.200)
    {
        cmd_desired_direction=0;
        cmd_linear_speed=0;
        cmd_angular_speed=0;
        cmd_pwm_gain=0;
        mov_timeout_counter++; 
    }
    if (wdt-wdt_joy_cmd > 0.200)
    {
        joy_desired_direction=0;
        joy_linear_speed=0;
        joy_angular_speed=0;
        joy_pwm_gain=0;
        joy_timeout_counter++;
    }
    if (wdt-wdt_aux_cmd > 0.200)
    {
        aux_desired_direction=0;
        aux_linear_speed=0;
        aux_angular_speed=0;
        aux_pwm_gain=0;
        aux_timeout_counter++;
    }
    if (wdt-wdt_ros_cmd > 0.200)
    {
        ros_desired_direction=0;
        ros_linear_speed=0;
        ros_angular_speed=0;
        ros_pwm_gain=0;
        ros_timeout_counter++;
    }

    if (wdt-wdt_old > 0.040) { thread_timeout_counter++;  }
    wdt_old=wdt;

    if (joystick_received>0)   { joystick_received--;  }
    if (command_received>0)    { command_received--;   }
    if (auxiliary_received>0)  { auxiliary_received--; }
    if (rosInput_received>0)   { rosInput_received--; }
}
