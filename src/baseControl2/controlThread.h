/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

#include "motors.h"
#include "input.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

enum robot_type_enum
{
    ROBOT_TYPE_NONE = 0,
    ROBOT_TYPE_DIFFERENTIAL = 1,
    ROBOT_TYPE_THREE_ROTOCASTER = 2,
    ROBOT_TYPE_THREE_MECHANUM = 3
};

enum control_type_enum
{
    BASE_CONTROL_NONE = 0,
    BASE_CONTROL_OPENLOOP_NO_PID = 1,
    BASE_CONTROL_OPENLOOP_PID = 2,
    BASE_CONTROL_VELOCITY_NO_PID = 3,
    BASE_CONTROL_VELOCITY_PID = 4
};

typedef iCub::ctrl::parallelPID parlPID;

class ControlThread : public yarp::os::PeriodicThread
{
private:
    Property             ctrl_options;
    double               thread_period;
    control_type_enum    base_control_type;
    int                  thread_timeout_counter;
                         
    //the current command
    double               input_linear_speed;
    double               input_angular_speed;
    double               input_desired_direction;
    double               input_pwm_gain;
                         
    //control pids
    parlPID*             linear_speed_pid;
    parlPID*             angular_speed_pid;
    parlPID*             linear_ol_pid;
    parlPID*             angular_ol_pid;
                         
    //controller parameters
    int                  robot_type;
    double               lin_ang_ratio;
    bool                 both_lin_ang_enabled;
    bool                 ratio_limiter_enabled;
    int                  input_filter_enabled;
    bool                 debug_enabled;
    double               max_angular_vel=0;
    double               max_linear_vel=0;
    double               max_angular_acc_pos = 0;
    double               max_angular_acc_neg = 0;
    double               max_linear_acc_pos = 0;
    double               max_linear_acc_neg = 0;

protected:
    ResourceFinder       &rf;
    PolyDriver           *control_board_driver;

    BufferedPort<Bottle> port_debug_linear;
    BufferedPort<Bottle> port_debug_angular;
    BufferedPort<Bottle> port_filtered_commands;
    BufferedPort<Bottle> port_unfiltered_commands;

    MotorControl*        m_motor_handler = nullptr;
    Input*               m_input_handler = nullptr;

    string               remoteName;
    string               localName;
    bool                 odometry_enabled;

public:
    //MotorControl and Input are instantiated by ControlThread.
    MotorControl* const  get_motor_handler()    { return m_motor_handler;}
    Input* const         get_input_handler()    { return m_input_handler; }
    void                 enable_debug(bool b);
    void                 set_max_ang_vel(double val) { max_angular_vel = val;}
    void                 set_max_lin_vel(double val) { max_linear_vel = val; }

public:
    /**
    * Constructor
    * @param _period the thread period, expressed in seconds.
    * @param _rf the resource finder containing the configuration options (from .ini file)
    * @param options additional configuration options
    */
    ControlThread   (double _period, ResourceFinder &_rf, Property options);

    //yarp::os::PeriodicThread methods
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();

public:

    /**
    * Sets the current control type
    * @param s can be one of the following: 'none', 'velocity_no_pid', 'openloop_no_pid', 'velocity_pid', 'openloop_pid'
    * @return true if the string is accepted
    */
    bool set_control_type (string s);
    
    /**
    * Return the current control type
    * @return the current robot control type
    */
    control_type_enum  get_control_type ();
    
    /**
    * Print stats about the current internal status (e.g. current control mode of the joints, etc).
    */
    void printStats();

    /**
    * Sets the PID control gains if the current control mode is: velocity_pid, openloop_pid.
    */
    void set_pid (string id, double kp, double ki, double kd);

    /**
    * Sets an low pass filter on input commands.
    * @param b is a code which specifies the filter type. Can be one of the following: 0 (disabled) or
    * 1,2,4,8 for a first order low-pass filter with a cut-off frequency of 1,2,4,8Hz.
    */
    void set_input_filter    (int b) {input_filter_enabled=b;}

    /**
    * Gets robot max linear velocity.
    * @return the maximum robot linear velocity, expressed in m/s
    */
    double get_max_linear_vel();

    /**
    * Gets robot max angular velocity.
    * @return the maximum robot angular velocity, expressed in deg/s
    */
    double get_max_angular_vel();

protected:
    //Perform filtering on control action. For internal use only.
    void apply_ratio_limiter (double max, double& linear_speed, double& angular_speed);
    void apply_ratio_limiter (double& linear_speed, double& angular_speed);
    void apply_acceleration_limiter (double& linear_speed, double& angular_speed, double& desired_direction);
    void apply_input_filter  (double& linear_speed, double& angular_speed, double& desired_direction);
    void apply_control_openloop_pid(double& pidout_linear_throttle, double& pidout_angular_throttle, const double ref_linear_speed, const double ref_angular_speed);
    void apply_control_speed_pid(double& pidout_linear_throttle, double& pidout_angular_throttle, const double ref_linear_speed, const double ref_angular_speed);
};

#endif
