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

#ifndef CONTROL_THREAD_H
#define CONTROL_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
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

#include "odometry.h"
#include "motors.h"
#include "input.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

enum
{
    ROBOT_TYPE_NONE = 0,
    ROBOT_TYPE_DIFFERENTIAL = 1,
    ROBOT_TYPE_THREE_ROTOCASTER = 2,
    ROBOT_TYPE_THREE_MECHANUM = 3
};

enum
{
    BASE_CONTROL_NONE = 0,
    BASE_CONTROL_OPENLOOP_NO_PID = 1,
    BASE_CONTROL_OPENLOOP_PID = 2,
    BASE_CONTROL_VELOCITY_NO_PID = 3,
    BASE_CONTROL_VELOCITY_PID = 4
};

class ControlThread : public yarp::os::RateThread
{
private:
    Property            ctrl_options;

    double              thread_period;
    int                 base_control_type;
    int                 thread_timeout_counter;

    //the current command
    double input_linear_speed;
    double input_angular_speed;
    double input_desired_direction;
    double input_pwm_gain;

    //movement control variables (internally computed)
    double              exec_linear_speed;
    double              exec_angular_speed;
    double              exec_desired_direction;
    double              exec_pwm_gain;

    //control pids
    iCub::ctrl::parallelPID*    linear_speed_pid;
    iCub::ctrl::parallelPID*    angular_speed_pid;
    iCub::ctrl::parallelPID*    linear_ol_pid;
    iCub::ctrl::parallelPID*    angular_ol_pid;

    //controller parameters
    int                 robot_type;
    double              lin_ang_ratio;
    bool                both_lin_ang_enabled;
    int                 input_filter_enabled;
    bool                debug_enabled;
    double              max_motor_pwm;
    double              max_motor_vel;

protected:
    ResourceFinder            &rf;
    PolyDriver                *control_board_driver;

    BufferedPort<Bottle>      port_debug_linear;
    BufferedPort<Bottle>      port_debug_angular;

    Odometry*                 odometry_handler;
    MotorControl*             motor_handler;
    Input*                    input_handler;

    string                    remoteName;
    string                    localName;

public:
    Odometry* const     get_odometry_handler() { return odometry_handler;}
    MotorControl* const get_motor_handler()    { return motor_handler;}
    Input* const get_input_handler()           { return input_handler; }
    void                enable_debug(bool b);

    ControlThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               ctrl_options(options)
    {
        control_board_driver     = 0;
        thread_timeout_counter   = 0;
        base_control_type        = BASE_CONTROL_NONE;

        input_filter_enabled     = 0;
        lin_ang_ratio            = 0.7;
        robot_type               = ROBOT_TYPE_NONE;

        debug_enabled            = false;
        both_lin_ang_enabled     = true;
        thread_period            = _period;

        input_linear_speed       = 0;
        input_angular_speed      = 0;
        input_desired_direction  = 0;
        input_pwm_gain           = 0;
        linear_speed_pid         = 0;
        angular_speed_pid        = 0;
        linear_ol_pid            = 0;
        angular_ol_pid           = 0;
        max_motor_pwm            = 0;
        remoteName               = ctrl_options.find("remote").asString();
        localName                = ctrl_options.find("local").asString();
    }

    virtual bool threadInit();

    virtual void afterStart(bool s)
    {
        if (s)
            yInfo("Control thread started successfully");
        else
            yError("Control thread did not start");
    }

    virtual void run();
    bool set_control_type (string s);
    int get_control_type ();
    void printStats();
    void set_pid (string id, double kp, double ki, double kd);
    void apply_ratio_limiter (double max, double& linear_speed, double& angular_speed);
    void apply_ratio_limiter (double& linear_speed, double& angular_speed);
    void apply_input_filter  (double& linear_speed, double& angular_speed, double& desired_direction);
    void set_input_filter    (int b) {input_filter_enabled=b;}
    
    void apply_control_openloop_pid(double& pidout_linear_speed,double& pidout_angular_speed, const double ref_linear_speed,const double ref_angular_speed);
    void apply_control_speed_pid(double& pidout_linear_speed,double& pidout_angular_speed, const double ref_linear_speed,const double ref_angular_speed);

    virtual void threadRelease()
    {
        if (odometry_handler)  {delete odometry_handler; odometry_handler=0;}
        if (motor_handler)     {delete motor_handler; motor_handler=0;}
        if (input_handler)     {delete input_handler; input_handler = 0; }

        if (linear_speed_pid)    {delete linear_speed_pid;  linear_speed_pid=0;}
        if (angular_speed_pid)   {delete angular_speed_pid; angular_speed_pid=0;}
        if (linear_ol_pid)    {delete linear_ol_pid;  linear_ol_pid=0;}
        if (angular_ol_pid)   {delete angular_ol_pid; angular_ol_pid=0;}

        if (debug_enabled)
        {
            port_debug_linear.interrupt();
            port_debug_linear.close();
            port_debug_angular.interrupt();
            port_debug_angular.close();
        }
    }
};

#endif
