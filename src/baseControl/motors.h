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

#ifndef MOTORS_CTRL_H
#define MOTORS_CTRL_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>
#include <vector>
#include <geometry_msgs_Twist.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#ifndef RAD2DEG
#define RAD2DEG 180.0/3.14159
#endif 

#ifndef DEG2RAD
#define DEG2RAD 3.14159/180.0
#endif

class MotorControl
{
protected:
    Property            ctrl_options;
    double              thread_period;
    std::vector<double> F;
    std::vector<int>    board_control_modes;
    std::vector<int>    board_control_modes_last;
    int                 thread_timeout_counter;

    double              max_motor_pwm;
    double              max_motor_vel;

protected:
    //ResourceFinder            rf;
    PolyDriver                *control_board_driver;

    int                motors_filter_enabled;
    string             localName;

    IPidControl       *ipid;
    IVelocityControl  *ivel;
    IEncoders         *ienc;
    IAmplifierControl *iamp;
    IPWMControl       *ipwm;
    IControlMode2     *icmd;

    //ros
    bool                                         enable_ROS;
    bool                                         enable_ROS_OUTPUT_GROUP;
    std::string                                  rosTopicName_cmd_twist;
    yarp::os::Publisher<geometry_msgs_Twist>     rosPublisherPort_cmd_twist;

public:

    MotorControl(unsigned int _period, PolyDriver* _driver);
    virtual ~MotorControl();
    virtual bool set_control_velocity() = 0;
    virtual bool set_control_openloop() = 0;
    virtual bool set_control_idle() = 0;

    virtual bool open(ResourceFinder &_rf, Property &_options);
    virtual void close();
    virtual void execute_none() = 0;
    virtual void execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;
    virtual void execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;
    virtual void decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;
    virtual bool check_motors_on() = 0;
    virtual void updateControlMode() = 0;
    virtual void printStats() = 0;

    virtual void set_motors_filter(int b) {motors_filter_enabled=b;}
    virtual double get_max_motor_vel()   {return max_motor_vel;}
    virtual double get_max_motor_pwm()   {return max_motor_pwm;}
    virtual void  apply_motor_filter(int i);
};

#endif
