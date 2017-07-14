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

#ifndef INPUT_H
#define INPUT_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Node.h>
#include <geometry_msgs_Twist.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;



class Input
{
public:
    struct InputDescription
    {
        enum InputType{BUTTON, AXIS, HAT};

        InputType    type;
        unsigned int Id;
        float        Factor;
        InputDescription() = default;
        InputDescription(unsigned int id, float factor) : Id(id), Factor(factor){}
    };

    struct JoyDescription
    {
        InputDescription xAxis;
        InputDescription yAxis;
        InputDescription tAxis;
        InputDescription gain;
        JoyDescription() = default;
        JoyDescription(InputDescription x, InputDescription y, InputDescription t, InputDescription g) :
            xAxis(x),
            yAxis(y),
            tAxis(t),
            gain(g)
        {}
    }jDescr;
private:
    Property ctrl_options;

    double              thread_period;

    int                 thread_timeout_counter;

    int                 command_received;
    int                 rosInput_received;
    int                 auxiliary_received;
    int                 joystick_received;

    int                 mov_timeout_counter;
    int                 aux_timeout_counter;
    int                 joy_timeout_counter;
    int                 ros_timeout_counter;

    //movement control variables (input from external)
    double              joy_linear_speed;
    double              joy_angular_speed;
    double              joy_desired_direction;
    double              joy_pwm_gain;

    double              cmd_linear_speed;
    double              cmd_angular_speed;
    double              cmd_desired_direction;
    double              cmd_pwm_gain;

    double              aux_linear_speed;
    double              aux_angular_speed;
    double              aux_desired_direction;
    double              aux_pwm_gain;
    
    double              ros_linear_speed;
    double              ros_angular_speed;
    double              ros_desired_direction;
    double              ros_pwm_gain;

    double              linear_vel_at_100_joy;
    double              angular_vel_at_100_joy;

    IJoypadController*  joypad;

protected:
    //ResourceFinder            rf;
    PolyDriver*                       control_board_driver;
    BufferedPort<Bottle>              port_movement_control;
    Subscriber<geometry_msgs_Twist>   rosSubscriberPort_twist;
    string                            rosTopicName_twist;
    bool                              useRos;
    bool                              enable_ROS_INPUT_GROUP;
    BufferedPort<Bottle>              port_auxiliary_control;
    BufferedPort<Bottle>              port_joystick_control;
    string                            localName;

public:

    Input(unsigned int _period, PolyDriver* _driver, IJoypadController* const joypad, JoyDescription joydesc);
    Input(unsigned int _period, PolyDriver* _driver);
    ~Input();

    bool   open(ResourceFinder &_rf, Property &_options);
    void   close();
    void   updateControlMode();
    void   printStats();
 
    void   read_inputs        (double *linear_speed, double *angular_speed, double *desired_direction, double *pwm_gain);
    void   read_percent_polar (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_percent_cart  (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_polar   (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_cart    (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_cart    (double x_speed, double y_speed, double& des_dir, double& lin_spd, double& pwm_gain);
    //double get_max_linear_vel()   {return max_linear_vel;}
    //double get_max_angular_vel()  {return max_angular_vel;}

    double get_linear_vel_at_100_joy()   { return linear_vel_at_100_joy; }
    double get_angular_vel_at_100_joy()  { return angular_vel_at_100_joy; }
};

#endif
