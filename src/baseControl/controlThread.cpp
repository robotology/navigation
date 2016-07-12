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

#include "controlThread.h"
#include "filters.h"
#include "cer/cer_odometry.h"
#include "ikart/ikart_odometry.h"
#include "cer/cer_motors.h"
#include "ikart/ikart_motors.h"

void ControlThread::apply_ratio_limiter (double& linear_speed, double& angular_speed)
{
    if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
    if (linear_speed>100)   linear_speed = 100;
    if (linear_speed<-100)  linear_speed = -100;
    if (angular_speed>100)  angular_speed = 100;
    if (angular_speed<-100) angular_speed = -100;

    double tot = fabs(linear_speed) + fabs(angular_speed);

    if (tot> 100)
    {
        //if lin_ang_ratio is negative, coeff will be 0 and
        //the adaptive limiter will be used (current ratio)
        double coeff = 0.0;
        if (lin_ang_ratio>0.0) coeff = (tot-100.0)/100.0;
        double angular_speed_A, angular_speed_B, linear_speed_A, linear_speed_B;

        angular_speed_A = angular_speed *(1-lin_ang_ratio);
        linear_speed_A  = linear_speed * lin_ang_ratio;

        double current_ratio = fabs(linear_speed/angular_speed);
        if (angular_speed>0) angular_speed_B =  100.0/(current_ratio+1.0);
        else                 angular_speed_B = -100.0/(current_ratio+1.0);

        linear_speed_B  =  100.0-fabs(angular_speed_B);
        //if (angular_speed>0) linear_speed_B  =  100.0-fabs(angular_speed_B);
        //else                 linear_speed_B  = -100.0+fabs(angular_speed_B);

        linear_speed  = linear_speed_A  *     (coeff) + linear_speed_B  * (1.0-coeff);
        angular_speed = angular_speed_A *     (coeff) + angular_speed_B * (1.0-coeff);

    }
}

void ControlThread::apply_ratio_limiter (double max, double& linear_speed, double& angular_speed)
{
    if (lin_ang_ratio<0.0)  lin_ang_ratio = 0.0;
    if (lin_ang_ratio>1.0)  lin_ang_ratio = 1.0;
    if (linear_speed  >  max*lin_ang_ratio) linear_speed  = max*lin_ang_ratio;
    if (linear_speed  < -max*lin_ang_ratio) linear_speed  = -max*lin_ang_ratio;
    if (angular_speed >  max*(1-lin_ang_ratio)) angular_speed = max*(1-lin_ang_ratio);
    if (angular_speed < -max*(1-lin_ang_ratio)) angular_speed = -max*(1-lin_ang_ratio);
}

void ControlThread::apply_input_filter (double& linear_speed, double& angular_speed, double& desired_direction)
{
    if (input_filter_enabled == 8)
    {
        angular_speed = control_filters::lp_filter_8Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_8Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_8Hz(desired_direction, 9);
    }
    if (input_filter_enabled == 4)
    {
        angular_speed = control_filters::lp_filter_4Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_4Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_4Hz(desired_direction, 9);
    }
    if (input_filter_enabled == 2)
    {
        angular_speed = control_filters::lp_filter_2Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_2Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_2Hz(desired_direction, 9);
    }
    if (input_filter_enabled == 1)
    {
        angular_speed = control_filters::lp_filter_1Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_1Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_1Hz(desired_direction, 9);
    }


}

void ControlThread::enable_debug(bool b)
{
    debug_enabled = b;
    if (b)
    {
        port_debug_linear.open((localName+"/debug/linear:o").c_str());
        port_debug_angular.open((localName+"/debug/angular:o").c_str());
    }
    else
    {
        port_debug_linear.interrupt();
        port_debug_linear.close();
        port_debug_angular.interrupt();
        port_debug_angular.close();
    }
}

void ControlThread::set_pid (string id, double kp, double ki, double kd)
{
    yarp::os::Bottle old_options;
    this->angular_speed_pid->getOptions(old_options);
    yInfo("Current configuration: %s\n",old_options.toString().c_str());
    
    // (Kp (10.0)) (Ki (0.0)) (Kf (0.0)) ... (satLim(-1000.0 1000.0)) (Ts 0.02)
    yarp::os::Bottle options;
    yarp::os::Bottle& bkp = options.addList();
    yarp::os::Bottle& bki = options.addList();
    yarp::os::Bottle& bkd = options.addList();
    bkp.addString("Kp");    yarp::os::Bottle& bkp2 = bkp.addList();    bkp2.addDouble(kp);
    bki.addString("Ki");    yarp::os::Bottle& bki2 = bki.addList();    bki2.addDouble(ki);
    bkd.addString("Kd");    yarp::os::Bottle& bkd2 = bkd.addList();    bkd2.addDouble(kd);
    yInfo("new configuration: %s\n", options.toString().c_str());

    this->angular_speed_pid->setOptions(options);
    yarp::sig::Vector tmp; tmp.resize(1); tmp.zero();
    this->angular_speed_pid->reset(tmp);
}

void ControlThread::apply_control_speed_pid(double& pidout_linear_speed,double& pidout_angular_speed, 
                           const double ref_linear_speed, const double ref_angular_speed)
{
    double feedback_linear_speed = this->odometry_handler->get_base_vel_lin() / this->motor_handler->get_max_linear_vel() * 200;
    double feedback_angular_speed = this->odometry_handler->get_base_vel_theta() / this->motor_handler->get_max_angular_vel() * 200;
    yarp::sig::Vector tmp;
    tmp = linear_speed_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
 //   pidout_linear_speed  = exec_pwm_gain * tmp[0];
    pidout_linear_speed  = 1.0 * tmp[0];
    tmp = angular_speed_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
   // pidout_angular_speed = exec_pwm_gain * tmp[0];
    pidout_angular_speed = 1.0 * tmp[0];
    //pidout_linear_speed=0; //@@@
    //pidout_angular_speed=0; //@@@

    if (debug_enabled) // debug block
    {
        char buff [255];
        if (port_debug_linear.getOutputCount()>0)
        {
            Bottle &b1=port_debug_linear.prepare();
            b1.clear();
            yInfo("%+9.4f %+9.4f %+9.4f %+9.4f",ref_linear_speed,feedback_linear_speed,ref_linear_speed-feedback_linear_speed,pidout_linear_speed);
            b1.addString(buff);
            port_debug_linear.write();
        }

        if (port_debug_angular.getOutputCount()>0)
        {
            Bottle &b2=port_debug_angular.prepare();
            b2.clear();
            yInfo("%+9.4f %+9.4f %+9.4f %+9.4f", ref_angular_speed, feedback_angular_speed, ref_angular_speed - feedback_angular_speed, pidout_angular_speed);
            b2.addString(buff);
            port_debug_angular.write();
        }
    }
}

void ControlThread::apply_control_openloop_pid(double& pidout_linear_speed,double& pidout_angular_speed, const double ref_linear_speed,const double ref_angular_speed)
{
    double feedback_linear_speed = this->odometry_handler->get_base_vel_lin() / this->motor_handler->get_max_linear_vel() * 100;
    double feedback_angular_speed = this->odometry_handler->get_base_vel_theta() / this->motor_handler->get_max_angular_vel() * 100;
    yarp::sig::Vector tmp;
    tmp = linear_ol_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
    pidout_linear_speed  = exec_pwm_gain * tmp[0];
    tmp = angular_ol_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
    pidout_angular_speed = exec_pwm_gain * tmp[0];
    pidout_linear_speed=0; //@@@
    pidout_angular_speed=0; //@@@

    if (debug_enabled) // debug block
    {
        char buff [255];
        Bottle &b1=port_debug_linear.prepare();
        b1.clear();
        yInfo( "%+9.4f %+9.4f %+9.4f %+9.4f",ref_linear_speed,feedback_linear_speed,ref_linear_speed-feedback_linear_speed,pidout_linear_speed);
        b1.addString(buff);
        port_debug_linear.write();

        Bottle &b2=port_debug_angular.prepare();
        b2.clear();
        yInfo( "%+9.4f %+9.4f %+9.4f %+9.4f", ref_angular_speed, feedback_angular_speed, ref_angular_speed - feedback_angular_speed, pidout_angular_speed);
        b2.addString(buff);
        port_debug_angular.write();
    }
}

void ControlThread::run()
{
    this->odometry_handler->compute();
    this->odometry_handler->broadcast();

    double pidout_linear_speed  = 0;
    double pidout_angular_speed = 0;
    double pidout_direction     = 0;

    //input_linear_speed and input_angular speed ranges are: 0-100
    this->input_handler->read_inputs(&input_linear_speed, &input_angular_speed, &input_desired_direction, &input_pwm_gain);
    apply_input_filter(input_linear_speed, input_angular_speed,input_desired_direction);
    apply_ratio_limiter(input_linear_speed, input_angular_speed);

    /*
    if (!lateral_movement_enabled)
    {
        if (input_desired_direction>-90 && input_desired_direction <90) input_desired_direction = 0;
        else if (input_desired_direction <= -90) input_desired_direction = 180;
        else if (input_desired_direction >= +90) input_desired_direction = 180;
    }
    */

    exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
    exec_desired_direction = input_desired_direction;

    //The controllers
    if (base_control_type == BASE_CONTROL_OPENLOOP_NO_PID)
    {
        exec_linear_speed  = input_linear_speed  / 100.0 * max_motor_pwm * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * max_motor_pwm * exec_pwm_gain;
        
        pidout_linear_speed  = exec_linear_speed;
        pidout_angular_speed = exec_angular_speed;
        pidout_direction     = exec_desired_direction;
        this->motor_handler->execute_openloop(pidout_linear_speed, pidout_direction, pidout_angular_speed);
    }
    else if (base_control_type == BASE_CONTROL_VELOCITY_NO_PID)
    {
        exec_linear_speed = input_linear_speed / 100.0 *  max_motor_vel * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * max_motor_vel * exec_pwm_gain;

//#define PRINT_CURRENT_VEL
#ifdef  PRINT_CURRENT_VEL
        //yDebug("%+5.5f, %+5.5f, %+5.5f\n", input_angular_speed /100 * this->motor_handler->get_max_angular_vel(), this->odometry_handler->base_vel_theta, exec_angular_speed);
        yDebug("%+5.5f, %+5.5f, %+5.5f\n", input_linear_speed /100 * this->motor_handler->get_max_linear_vel(), this->odometry_handler->base_vel_lin, exec_linear_speed);
#endif

        pidout_linear_speed  = exec_linear_speed;
        pidout_angular_speed = exec_angular_speed;
        pidout_direction     = exec_desired_direction;
        this->motor_handler->execute_speed(pidout_linear_speed, pidout_direction, pidout_angular_speed);
    }
    else if (base_control_type == BASE_CONTROL_OPENLOOP_PID)
    {
        exec_linear_speed = input_linear_speed / 100.0 * max_motor_pwm * exec_pfwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 * max_motor_pwm * exec_pwm_gain;
        
        apply_control_openloop_pid(pidout_linear_speed,pidout_angular_speed,exec_linear_speed,exec_angular_speed);
        this->motor_handler->execute_speed(pidout_linear_speed, pidout_direction, pidout_angular_speed);
    }
    else if (base_control_type == BASE_CONTROL_VELOCITY_PID)
    {
        const double max_wheels_vel = 200;
        exec_linear_speed = input_linear_speed / 100.0 * max_wheels_vel * exec_pwm_gain;
        exec_angular_speed = input_angular_speed / 100.0 *  max_wheels_vel * exec_pwm_gain;
        //exec_linear_speed = input_linear_speed / 100.0 *  this->motor_handler->get_max_linear_vel() * exec_pwm_gain;
        //exec_angular_speed = input_angular_speed / 100.0 *  this->motor_handler->get_max_angular_vel() * exec_pwm_gain;
        
        apply_control_speed_pid(pidout_linear_speed,pidout_angular_speed,exec_linear_speed,exec_angular_speed);
        
        pidout_angular_speed += exec_angular_speed;
        this->motor_handler->execute_speed(pidout_linear_speed, pidout_direction, pidout_angular_speed);
    }
    else
    {
        yError ("Unknown control mode!");
        exec_linear_speed = 0;
        exec_angular_speed = 0;
        exec_pwm_gain = 0;
        exec_desired_direction = 0;
        this->motor_handler->execute_none();
    }
}

void ControlThread::printStats()
{
    yInfo ("* Control thread:\n");
    yInfo ("Input command: %+5.0f %+5.0f %+5.0f  %+5.0f      ", input_linear_speed, input_angular_speed, input_desired_direction, input_pwm_gain);
    yInfo ("Robot command: %+5.2f %+5.2f\n", input_linear_speed / 100.0*this->get_motor_handler()->get_max_linear_vel(), input_angular_speed / 100.0*this->get_motor_handler()->get_max_angular_vel());
}

bool ControlThread::set_control_type (string s)
{
    if      (s == "none")            base_control_type = BASE_CONTROL_NONE;
    else if (s == "velocity_no_pid") base_control_type = BASE_CONTROL_VELOCITY_NO_PID;
    else if (s == "openloop_no_pid") base_control_type = BASE_CONTROL_OPENLOOP_NO_PID;
    else if (s == "velocity_pid")    base_control_type = BASE_CONTROL_VELOCITY_PID;
    else if (s == "openloop_pid")    base_control_type = BASE_CONTROL_OPENLOOP_PID;
    else
    {
        yError("Error: unknown type of control required: %s. Closing...\n",s.c_str());
        return false;
    }
    yInfo("Control type set to: %s\n",s.c_str());
    return true;
}

int ControlThread::get_control_type ()
{
    return base_control_type;
}

bool ControlThread::threadInit()
{
    if (!ctrl_options.check("GENERAL"))
    {
        yError() << "Missing [GENERAL] section";
        return false;
    }
    yarp::os::Bottle& general_options = ctrl_options.findGroup("GENERAL");
    if (general_options.check("control_mode") == false) { yError() << "Missing 'control_mode' param"; return false; }
    if (general_options.check("input_filter_enabled") == false) { yError() << "Missing 'input_filter_enabled' param"; return false; }
    if (general_options.check("linear_angular_ratio") == false) { yError() << "Missing 'linear_angular_ratio' param"; return false; }
    if (general_options.check("max_motor_pwm") == false) { yError() << "Missing 'max_motor_pwm' param"; return false; }
    if (general_options.check("max_motor_vel") == false) { yError() << "Missing 'max_motor_vel' param"; return false; }
    if (general_options.check("robot_type") == false) { yError() << "Missing 'robot_type' param"; return false; }
    string control_type = general_options.check("control_mode", Value("none"), "type of control for the wheels").asString().c_str();
    input_filter_enabled = general_options.check("input_filter_enabled", Value(0), "input filter frequency (1/2/4/8Hz, 0 = disabled)").asInt();
    lin_ang_ratio = general_options.check("linear_angular_ratio", Value(0.7), "ratio (<1.0) between the maximum linear speed and the maximum angular speed.").asDouble();
    max_motor_pwm = general_options.check("max_motor_pwm", Value(0), "max_motor_pwm").asDouble();
    max_motor_vel = general_options.check("max_motor_vel", Value(0), "max_motor_vel").asDouble();
    string robot_type_s = general_options.check("robot_type", Value("none"), "geometry of the robot").asString();

    // open the control board driver
    yInfo("Opening the motors interface...\n");
    int trials = 0;
    double start_time = yarp::os::Time::now();
    Property control_board_options("(device remote_controlboard)");
    control_board_options.put("remote", remoteName.c_str());
    control_board_options.put("local", localName.c_str());

    do
    {
        double current_time = yarp::os::Time::now();

        //remove previously existing drivers
        if (control_board_driver)
        {
            delete control_board_driver;
            control_board_driver = 0;
        }

        //creates the new device driver
        control_board_driver = new PolyDriver(control_board_options);
        bool connected = control_board_driver->isValid();

        //check if the driver is connected
        if (connected) break;

        //check if the timeout (10s) is expired
        if (current_time - start_time > 10.0)
        {
            yError("It is not possible to instantiate the device driver. I tried %d times!", trials);
            if (control_board_driver)
            {
                delete control_board_driver;
                control_board_driver = 0;
            }
            return false;
        }

        yarp::os::Time::delay(0.5);
        trials++;
        yWarning("Unable to connect the device driver, trying again...");
    } while (true);

    //create the odometry and the motor handlers

    if (robot_type_s == "cer")
    {
        yInfo("Using cer robot type");
        robot_type = ROBOT_TYPE_DIFFERENTIAL;
        odometry_handler = new CER_Odometry((int)(thread_period), control_board_driver);
        motor_handler = new CER_MotorControl((int)(thread_period), control_board_driver);
        input_handler = new Input((int)(thread_period), control_board_driver);
    }
    else if (robot_type_s == "ikart_V1")
    {
        yInfo("Using ikart_V1 robot type");
        robot_type = ROBOT_TYPE_THREE_ROTOCASTER;
        odometry_handler = new iKart_Odometry((int)(thread_period), control_board_driver);
        motor_handler = new iKart_MotorControl((int)(thread_period), control_board_driver);
        input_handler = new Input((int)(thread_period), control_board_driver);
    }
    else if (robot_type_s == "ikart_V2")
    {
        yInfo("Using ikart_V2 robot type");
        robot_type = ROBOT_TYPE_THREE_MECHANUM;
        odometry_handler = new iKart_Odometry((int)(thread_period), control_board_driver);
        motor_handler = new iKart_MotorControl((int)(thread_period), control_board_driver);
        input_handler = new Input((int)(thread_period), control_board_driver);
    }
    else
    {
        yError() << "Invalid Robot type selected: ROBOT_TYPE_NONE";
        return false;
    }
    
    if (odometry_handler->open(rf, ctrl_options) == false)
    {
        yError() << "Problem occurred while opening odometry handler";
        return false;
    }

    if (motor_handler->open(rf, ctrl_options) == false)
    {
        yError() << "Problem occurred while opening motor handler";
        return false;
    }

    if (input_handler->open(rf, ctrl_options) == false)
    {
        yError() << "Problem occurred while opening input handler";
        return false;
    }

    yInfo("%s", ctrl_options.toString().c_str());

    //create the pid controllers
    if (!ctrl_options.check("HEADING_VELOCITY_PID"))
    {
        yError("Error reading from .ini file, section PID");
        return false;
    }
    if (!ctrl_options.check("LINEAR_VELOCITY_PID"))
    {
        yError("Error reading from .ini file, section PID");
        return false;
    }
    if (!ctrl_options.check("ANGULAR_VELOCITY_PID"))
    {
        yError("Error reading from .ini file, section PID");
        return false;
    }
    yarp::sig::Vector kp[3], ki[3], kd[3];
    yarp::sig::Vector wp[3], wi[3], wd[3];
    yarp::sig::Vector N[3];
    yarp::sig::Vector Tt[3];
    yarp::sig::Matrix sat[3];
    for (int i = 0; i<3; i++)
    {
        kp[i].resize(1); ki[i].resize(1); kd[i].resize(1);
        kp[i] = ki[i] = kd[i] = 0.0;
        wp[i].resize(1); wi[i].resize(1); wd[i].resize(1);
        wp[i] = wi[i] = wd[i] = 1.0;
        N[i].resize(1); Tt[i].resize(1); sat[i].resize(1, 2);
        N[i] = 10;
        Tt[i] = 1;
    }

    kp[0] = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("kp", Value(0), "kp gain").asDouble();
    kd[0] = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("kd", Value(0), "kd gain").asDouble();
    ki[0] = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("ki", Value(0), "ki gain").asDouble();
    sat[0](0, 0) = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("min", Value(0), "min").asDouble();
    sat[0](0, 1) = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("max", Value(0), "max").asDouble();

    kp[1] = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("kp", Value(0), "kp gain").asDouble();
    kd[1] = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("kd", Value(0), "kd gain").asDouble();
    ki[1] = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("ki", Value(0), "ki gain").asDouble();
    sat[1](0, 0) = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("min", Value(0), "min").asDouble();
    sat[1](0, 1) = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("max", Value(0), "max").asDouble();

    linear_speed_pid = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[0], ki[0], kd[0], wp[0], wi[0], wd[0], N[0], Tt[0], sat[0]);
    angular_speed_pid = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[1], ki[1], kd[1], wp[1], wi[1], wd[1], N[1], Tt[1], sat[1]);

    linear_ol_pid = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[0], ki[0], kd[0], wp[0], wi[0], wd[0], N[0], Tt[0], sat[0]);
    angular_ol_pid = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[1], ki[1], kd[1], wp[1], wi[1], wd[1], N[1], Tt[1], sat[1]);

    //debug ports
    if (debug_enabled)
    {
        port_debug_linear.open((localName + "/debug/linear:o").c_str());
        port_debug_angular.open((localName + "/debug/angular:o").c_str());
    }

    //start the motors
    if (rf.check("no_start"))
    {
        yInfo("no_start option found");
        return true;
    }

    yInfo() << control_type.c_str();
    if      (control_type == string("velocity_pid"))    { this->set_control_type("velocity_pid");    yInfo("setting control mode velocity");  this->get_motor_handler()->set_control_velocity(); return true; }
    else if (control_type == string("velocity_no_pid")) { this->set_control_type("velocity_no_pid"); yInfo("setting control mode velocity");  this->get_motor_handler()->set_control_velocity(); return true; }
    else if (control_type == string("openloop_pid"))    { this->set_control_type("openloop_pid");    yInfo("setting control mode openloop");  this->get_motor_handler()->set_control_openloop(); return true; }
    else if (control_type == string("openloop_no_pid")) { this->set_control_type("openloop_no_pid"); yInfo("setting control mode openloop");  this->get_motor_handler()->set_control_openloop(); return true; }
    else if (control_type == string("none"))            { this->set_control_type("none");            yInfo("setting control mode none");  return true; }
    else                                              { yError("Invalid control_mode");  return false; }
}
