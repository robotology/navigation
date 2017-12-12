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

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif 

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

class MotorControl
{
protected:
    int                 motors_num;
    Property            ctrl_options;
    std::vector<double> F;
    std::vector<int>    board_control_modes;
    std::vector<int>    board_control_modes_last;
    int                 thread_timeout_counter;

    double              max_motor_pwm;
    double              max_motor_vel;

public:
    enum filter_frequency {DISABLED=0, HZ_05=1, HZ_1=2, HZ_2=3, HZ_4=4, HZ_8=5};

protected:
    filter_frequency           motors_filter_enabled;
    string                     localName;
    BufferedPort<Bottle>       port_status;

    //motor control interfaces
    PolyDriver            *control_board_driver;
    IPidControl           *ipid;
    IVelocityControl      *ivel;
    IEncoders             *ienc;
    IAmplifierControl     *iamp;
    IPWMControl           *ipwm;
    IControlMode2         *icmd;

    //ROS
    bool                                         enable_ROS;
    bool                                         enable_ROS_OUTPUT_GROUP;
    std::string                                  rosTopicName_cmd_twist;
    yarp::os::Publisher<geometry_msgs_Twist>     rosPublisherPort_cmd_twist;

protected:
    /**
    * Decouples the control, i.e. computes the individual motor commands, given the robot velocity command in the cartesian space.
    * @param appl_linear_speed the mobile base linear speed
    * @param appl_desired_direction the mobile base heading (appl_linear_speed will be applied taking in account robot reference frame).
    * @param appl_angular_speed the mobile base angular speed (in-place rotation).
    */
    virtual void decouple(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;

public:
    /**
    * Constructor
    * @param _driver is a pointer to a remoteControlBoard driver.
    */
    MotorControl(PolyDriver* _driver);

    /**
    * Destructor
    */
    virtual ~MotorControl();

    /**
    * Sets all the robot joints to control mode velocity. BaseControl will send velocity reference to motor joints
    * @return true if the operation was successful, false otherwise.
    */
    virtual bool set_control_velocity();

    /**
    * Sets all the robot joints to control mode openloop. BaseControl will send openloop commands to motor joints
    * @return true if the operation was successful, false otherwise.
    */
    virtual bool set_control_openloop();

    /**
    * Sets all the robot joints to control mode idle.
    * @return true if the operation was successful, false otherwise.
    */
    virtual bool set_control_idle();

    /**
    * Checks if robot joints are not idle or faulted.
    * @return true if the motor control is active, false otherwise.
    */
    virtual bool check_motors_on();

    /**
    * Checks if the system is fully functional. If a joint is in fault, turn off the control on all other joints.
    */
    virtual void updateControlMode();
   
    /**
    * Print stats about the current internal status (e.g. current control mode of the joints, etc).
    */
    virtual void printStats();

    /**
    * Opens the motors module, parsing the given options
    * @param _options the configuration option for the module
    * @return true if the motor module opened successfully. False if a mandatory parameter is missing or invalid.
    */
    virtual bool open(Property &_options);

    /**
    * Closes the motors module.
    */
    virtual void close();

    /**
    * Do not perform any control action. Simply keep the robot joint stopped.
    */
    virtual void execute_none() = 0;

    /**
    * Performs the control action, controlling the robot joints in openloop mode to track the required cartesian velocity commands.
    * @param appl_linear_speed the mobile base linear speed
    * @param appl_desired_direction the mobile base heading (appl_linear_speed will be applied taking in account robot reference frame).
    * @param appl_angular_speed the mobile base angular speed (in-place rotation).
    */
    virtual void execute_openloop(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;
    
    /**
    * Performs the control action, controlling the robot joints in velocity mode to track the required cartesian velocity commands.
    * @param appl_linear_speed the mobile base linear speed
    * @param appl_desired_direction the mobile base heading (appl_linear_speed will be applied taking in account robot reference frame).
    * @param appl_angular_speed the mobile base angular speed (in-place rotation).
    */
    virtual void execute_speed(double appl_linear_speed, double appl_desired_direction, double appl_angular_speed) = 0;
    
    /**
    * Enable/Disable/Sets the frequency of the motor output low pass filter. Filtering is performed by apply_motor_filter()
    * @param freq the low pass filter frequency (or filter_frequency::DISABLED) to turn off the filter
    */
    virtual void set_motors_filter(filter_frequency freq) {motors_filter_enabled=freq;}

    /**
    * Return the maximum value of joint velocity, as defined in the configuration parameters.
    * @return the maximum value of joint velocity. joint the joint number
    */
    virtual double get_max_motor_vel()   {return max_motor_vel;}
   
    /**
    * Return the maximum value of motor pwm, as defined in the configuration parameters.
    * @return the maximum value of motor pwm. joint the joint number
    */
    virtual double get_max_motor_pwm()   {return max_motor_pwm;}

    /**
    * Apply a low pass filter to motor output. The frequency is defined by motors_filter_enabled
    * @param joint the joint number
    */
    virtual void  apply_motor_filter(int joint);
};

#endif
