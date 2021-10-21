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
#include <yarp/rosmsg/geometry_msgs/Twist.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/MobileBaseVelocity.h>
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
    };

private:
    Property            ctrl_options;
    string              localName;
    int                 thread_timeout_counter;

    int                 command_received;
    int                 rosInput_received;
    int                 auxiliary_received;
    int                 joystick_received[2];
    JoyDescription      jDescr[2];

    //watchdog counters
    int                 mov_timeout_counter;
    int                 aux_timeout_counter;
    int                 joy_timeout_counter[2];
    int                 ros_timeout_counter;

    //movement control variables: joypad input
    double              joy_linear_speed[2];
    double              joy_angular_speed[2];
    double              joy_desired_direction[2];
    double              joy_pwm_gain[2];
public:
    double              linear_vel_at_100_joy;
    double              angular_vel_at_100_joy;

private:
    //standard input via YARP port
    double              cmd_linear_speed;
    double              cmd_angular_speed;
    double              cmd_desired_direction;
    double              cmd_pwm_gain;

    //aux input via YARP port
    double              aux_linear_speed;
    double              aux_angular_speed;
    double              aux_desired_direction;
    double              aux_pwm_gain;
    
    //ROS input
    double              ros_linear_speed;
    double              ros_angular_speed;
    double              ros_desired_direction;
    double              ros_pwm_gain;

protected:
    // ROS input
    Subscriber<yarp::rosmsg::geometry_msgs::Twist>   rosSubscriberPort_twist;
    string                            rosTopicName_twist;
    bool                              useRos;
    bool                              rosInputEnabled;

    // YARP ports input
    BufferedPort<Bottle>                          port_movement_control;
    BufferedPort<yarp::dev::MobileBaseVelocity>   port_auxiliary_control;
    BufferedPort<Bottle>*                         port_joystick_control[2];

    //Joypad input
    PolyDriver                        joyPolyDriver[2];
    IJoypadController*                iJoy[2];

public:

    /**
    * Default Constructor
    */
    Input();

    /**
    * Destructor
    */
    ~Input();

    /**
    * Opens the input module, parsing the given options
    * @param _options the configuration option for the module
    * @return true if the motor module opened successfully. False if a mandatory parameter is missing or invalid.
    */
    bool   open(Property &_options);

    /**
    * Closes the input module.
    */
    void   close();

    /**
    * Print some stats about the received input commands
    */
    void   printStats();
 
    /**
    * Receives the user input commands inputs. These source of these inputs may be a YARP port, a device (joypad), a ROS topic etc.
    * The robot commands are expressed as cartesian velocity in the robot reference frame.
    * @param linear_speed the mobile base linear speed
    * @param desired_direction the mobile base heading (linear_speed will be applied taking in account robot reference frame).
    * @param angular_speed the mobile base angular speed (in-place rotation).
    * @param pwm_gain the pwm gain (0-100). Joypad emergency button typically sets this value to zero to stop the robot. User modules, instead, do not use this value (always set to 100)/
    */
    void   read_inputs        (double& linear_speed, double& angular_speed, double& desired_direction, double& pwm_gain);
    
private:

    /**
    * Configures a joypad.
    * @param n the number of the joypad to be configured (0-1)
    * @param joypad_group the bottle containing the configuration options
    * @return true/false if the joypad has been successfully configured
    */
    bool   configureJoypdad   (int n, const Bottle& joypad_group);

    //Internal functions to extract velocity commands from a given bottle or from a joypad descriptor
    void   read_percent_polar (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_percent_cart  (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_polar   (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_cart    (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_joystick_data (JoyDescription *jDescr,IJoypadController* iJoy, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);

    //Performs conversion from joypad stick units to metric units
    double get_linear_vel_at_100_joy()   { return linear_vel_at_100_joy; }
    double get_angular_vel_at_100_joy()  { return angular_vel_at_100_joy; }
};

#endif
