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

void ControlThread::afterStart(bool s)
{
    if (s)
        yInfo("Control thread started successfully");
    else
        yError("Control thread did not start");
}

void ControlThread::threadRelease()
{
    if (odometry_handler)     {delete odometry_handler; odometry_handler=0;}
    if (odometry_handler_aux) {delete odometry_handler_aux; odometry_handler_aux = 0;}
    if (motor_handler)        {delete motor_handler; motor_handler=0;}
    if (input_handler)        {delete input_handler; input_handler = 0; }
                              
    if (linear_speed_pid)     {delete linear_speed_pid;  linear_speed_pid=0;}
    if (angular_speed_pid)    {delete angular_speed_pid; angular_speed_pid=0;}
    if (linear_ol_pid)        {delete linear_ol_pid;  linear_ol_pid=0;}
    if (angular_ol_pid)       {delete angular_ol_pid; angular_ol_pid=0;}

    if (debug_enabled)
    {
        port_debug_linear.interrupt();
        port_debug_linear.close();
        port_debug_angular.interrupt();
        port_debug_angular.close();
    }

    if (rosNode)
    {
        rosNode->interrupt();
        delete rosNode;
        rosNode = 0;
    }
}

double ControlThread::get_max_linear_vel()  { return max_linear_vel; }
double ControlThread::get_max_angular_vel() { return max_angular_vel; }

ControlThread::ControlThread (double _period, ResourceFinder &_rf, Property options) : PeriodicThread(_period), rf(_rf), ctrl_options(options)
{
    rosNode                  = NULL;
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
    max_linear_vel           = 0;
    max_angular_vel          = 0;
    max_angular_acc          = 0;
    max_linear_acc           = 0;
    remoteName               = ctrl_options.find("remote").asString();
    localName                = ctrl_options.find("local").asString();
    odometry_handler         = 0;
    odometry_handler_aux     = 0;
    motor_handler            = 0;
    input_handler            = 0;
}

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

        angular_speed = angular_speed *(1-lin_ang_ratio);
        linear_speed  = linear_speed * lin_ang_ratio;
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

void ControlThread::apply_acceleration_limiter(double& linear_speed, double& angular_speed, double& desired_direction)
{
    double period = this->getPeriod();
    angular_speed = control_filters::ratelim_filter_0(angular_speed, 7, max_angular_acc*period);
#if 0
    linear_speed = control_filters::ratelim_filter_0(linear_speed, 8, max_linear_acc*period);
    //the following line is not numerically correct because of max_linear_acc, but it prevents jerky motions
    desired_direction = control_filters::ratelim_filter_0(desired_direction, 9, max_linear_acc*period);
#else
    double xcomp = linear_speed * sin(desired_direction*DEG2RAD);
    double ycomp = linear_speed * cos(desired_direction*DEG2RAD);
    xcomp = control_filters::ratelim_filter_0(xcomp, 8, max_linear_acc*period);
    ycomp = control_filters::ratelim_filter_0(ycomp, 9, max_linear_acc*period);
    linear_speed = sqrt(xcomp * xcomp+ ycomp * ycomp);
    desired_direction = atan2(xcomp, ycomp) * 180.0 / M_PI;
#endif

    #if DEBUG_LIMTER
    yDebug()<<angular_speed<<linear_speed;
    #endif
}

void ControlThread::apply_input_filter (double& linear_speed, double& angular_speed, double& desired_direction)
{
    if (input_filter_enabled == 8)
    {
        angular_speed = control_filters::lp_filter_8Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_8Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_8Hz(desired_direction, 9);
    }
    else if (input_filter_enabled == 4)
    {
        angular_speed = control_filters::lp_filter_4Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_4Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_4Hz(desired_direction, 9);
    }
    else if(input_filter_enabled == 2)
    {
        angular_speed = control_filters::lp_filter_2Hz(angular_speed, 7);
        linear_speed = control_filters::lp_filter_2Hz(linear_speed, 8);
        desired_direction = control_filters::lp_filter_2Hz(desired_direction, 9);
    }
    else if(input_filter_enabled == 1)
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

void ControlThread::apply_control_speed_pid(double& pidout_linear_throttle,double& pidout_angular_throttle, 
                           const double ref_linear_speed, const double ref_angular_speed)
{
    double feedback_linear_speed = 0;
    double feedback_angular_speed = 0;
    if (odometry_handler) feedback_linear_speed = this->odometry_handler->get_base_vel_lin();
    if (odometry_handler) feedback_angular_speed = this->odometry_handler->get_base_vel_theta();
    yarp::sig::Vector tmp;
    tmp = linear_speed_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
    pidout_linear_throttle = 1.0 * tmp[0];
    tmp = angular_speed_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
    pidout_angular_throttle = 1.0 * tmp[0];

    if (debug_enabled) // debug block
    {
        char buff [255];
        if (port_debug_linear.getOutputCount()>0)
        {
            Bottle &b1=port_debug_linear.prepare();
            b1.clear();
            yInfo("%+9.4f %+9.4f %+9.4f %+9.4f", ref_linear_speed, feedback_linear_speed, ref_linear_speed - feedback_linear_speed, pidout_linear_throttle);
            b1.addString(buff);
            port_debug_linear.write();
        }

        if (port_debug_angular.getOutputCount()>0)
        {
            Bottle &b2=port_debug_angular.prepare();
            b2.clear();
            yInfo("%+9.4f %+9.4f %+9.4f %+9.4f", ref_angular_speed, feedback_angular_speed, ref_angular_speed - feedback_angular_speed, pidout_angular_throttle);
            b2.addString(buff);
            port_debug_angular.write();
        }
    }
}

void ControlThread::apply_control_openloop_pid(double& pidout_linear_throttle, double& pidout_angular_throttle, const double ref_linear_speed, const double ref_angular_speed)
{
    double feedback_linear_speed = 0; 
    double feedback_angular_speed = 0; 
    if (odometry_handler) feedback_linear_speed = this->odometry_handler->get_base_vel_lin();
    if (odometry_handler) feedback_angular_speed = this->odometry_handler->get_base_vel_theta();
    yarp::sig::Vector tmp;
    tmp = linear_ol_pid->compute(yarp::sig::Vector(1,ref_linear_speed),yarp::sig::Vector(1,feedback_linear_speed));
    pidout_linear_throttle = 1.0 * tmp[0];
    tmp = angular_ol_pid->compute(yarp::sig::Vector(1,ref_angular_speed),yarp::sig::Vector(1,feedback_angular_speed));
    pidout_angular_throttle = 1.0 * tmp[0];

    if (debug_enabled) // debug block
    {
        char buff [255];
        Bottle &b1=port_debug_linear.prepare();
        b1.clear();
        yInfo("%+9.4f %+9.4f %+9.4f %+9.4f", ref_linear_speed, feedback_linear_speed, ref_linear_speed - feedback_linear_speed, pidout_linear_throttle);
        b1.addString(buff);
        port_debug_linear.write();

        Bottle &b2=port_debug_angular.prepare();
        b2.clear();
        yInfo("%+9.4f %+9.4f %+9.4f %+9.4f", ref_angular_speed, feedback_angular_speed, ref_angular_speed - feedback_angular_speed, pidout_angular_throttle);
        b2.addString(buff);
        port_debug_angular.write();
    }
}

void ControlThread::run()
{
    if (odometry_handler) this->odometry_handler->compute();
    if (odometry_handler) this->odometry_handler->broadcast();
    if (odometry_handler_aux) this->odometry_handler_aux->compute();
    if (odometry_handler_aux) this->odometry_handler_aux->broadcast();

    double pidout_linear_throttle = 0;
    double pidout_angular_throttle = 0;
    double pidout_direction     = 0;

    //read inputs (input_linear_speed in m/s, input_angular_speed in deg/s...)
    this->input_handler->read_inputs(input_linear_speed, input_angular_speed, input_desired_direction, input_pwm_gain);

    if (input_linear_speed < 0)
    {
        static int print_counter = 0;
        if (print_counter++ == 10)
        {
            yWarning() << "Performance warning: input_angular_speed <0. This should not happen!";
            print_counter = 0;
        }
        input_linear_speed = -input_linear_speed;
        input_desired_direction += 180.0;
        if (input_desired_direction >= 360.0) input_desired_direction -= 360.0;
    }

    //low pass filter
    apply_input_filter(input_linear_speed, input_angular_speed, input_desired_direction);

    //acceleration_limiter
    apply_acceleration_limiter(input_linear_speed, input_angular_speed, input_desired_direction);

    //apply limiter
    if (input_linear_speed  > get_max_linear_vel())   input_linear_speed  = get_max_linear_vel();
    if (input_linear_speed  < -get_max_linear_vel())  input_linear_speed  = -get_max_linear_vel();
    if (input_angular_speed > get_max_angular_vel())  input_angular_speed = get_max_angular_vel();
    if (input_angular_speed < -get_max_angular_vel()) input_angular_speed = -get_max_angular_vel();
    
    //apply ratio limiter
    if (ratio_limiter_enabled) apply_ratio_limiter(input_linear_speed, input_angular_speed);

    /*
    if (!lateral_movement_enabled)
    {
        if (input_desired_direction>-90 && input_desired_direction <90) input_desired_direction = 0;
        else if (input_desired_direction <= -90) input_desired_direction = 180;
        else if (input_desired_direction >= +90) input_desired_direction = 180;
    }
    */

    //The controllers
    if (base_control_type == BASE_CONTROL_OPENLOOP_NO_PID)
    {
        double exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
        //the following /2 is used to avoid saturation due to decoupling
        pidout_linear_throttle = input_linear_speed / this->max_linear_vel * this->motor_handler->get_max_motor_pwm()/2 * exec_pwm_gain;
        pidout_angular_throttle = input_angular_speed / this->max_angular_vel * this->motor_handler->get_max_motor_pwm()/2 * exec_pwm_gain;
        pidout_direction = input_desired_direction;
        this->motor_handler->execute_openloop(pidout_linear_throttle, pidout_direction, pidout_angular_throttle);
    }
    else if (base_control_type == BASE_CONTROL_VELOCITY_NO_PID)
    {
        double exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
        pidout_linear_throttle = input_linear_speed * exec_pwm_gain;
        pidout_angular_throttle = input_angular_speed * exec_pwm_gain;
        pidout_direction     = input_desired_direction;
        this->motor_handler->execute_speed(pidout_linear_throttle, pidout_direction, pidout_angular_throttle);
    }
    else if (base_control_type == BASE_CONTROL_OPENLOOP_PID)
    {
        double exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
        apply_control_openloop_pid(pidout_linear_throttle, pidout_angular_throttle,
            (input_linear_speed * exec_pwm_gain),
            (input_angular_speed * exec_pwm_gain));
        this->motor_handler->execute_speed(pidout_linear_throttle, pidout_direction, pidout_angular_throttle);
    }
    else if (base_control_type == BASE_CONTROL_VELOCITY_PID)
    {
        double exec_pwm_gain = input_pwm_gain / 100.0 * 1.0;
        apply_control_speed_pid(pidout_linear_throttle, pidout_angular_throttle,
            (input_linear_speed * exec_pwm_gain),
            (input_angular_speed * exec_pwm_gain));
        this->motor_handler->execute_speed(pidout_linear_throttle, pidout_direction, pidout_angular_throttle);
    }
    else
    {
        yError ("Unknown control mode!");
        this->motor_handler->execute_none();
}
}

void ControlThread::printStats()
{
    yInfo ("* Control thread:\n");
    yInfo ("Input command: %+5.2f %+5.2f %+5.2f  %+5.2f      ", input_linear_speed, input_angular_speed, input_desired_direction, input_pwm_gain);
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

control_type_enum ControlThread::get_control_type ()
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
    if (general_options.check("ratio_limiter_enabled") == false) { yError() << "Missing 'ratio_limiter_enabled' param"; return false; }
    if (general_options.check("input_filter_enabled") == false) { yError() << "Missing 'input_filter_enabled' param"; return false; }
    if (general_options.check("linear_angular_ratio") == false) { yError() << "Missing 'linear_angular_ratio' param"; return false; }
    if (general_options.check("robot_type") == false) { yError() << "Missing 'robot_type' param"; return false; }
    if (general_options.check("max_linear_vel") == false) { yError() << "Missing 'max_linear_vel' param";  return false; }
    if (general_options.check("max_angular_vel") == false) { yError() << "Missing 'max_angular_vel' param";   return false; }
    if (general_options.check("max_linear_acc") == false) { yError() << "Missing 'max_linear_acc' param";  return false; }
    if (general_options.check("max_angular_acc") == false) { yError() << "Missing 'max_angular_acc' param";   return false; }

    string control_type, robot_type_s;
    bool useRos;

    control_type = general_options.check("control_mode", Value("none"), "type of control for the wheels").asString().c_str();
    input_filter_enabled = general_options.check("input_filter_enabled", Value(0), "input filter frequency (1/2/4/8Hz), 0 = disabled)").asInt();
    ratio_limiter_enabled = general_options.check("ratio_limiter_enabled", Value(0), "1=enabled, 0 = disabled").asInt() == 1;
    lin_ang_ratio = general_options.check("linear_angular_ratio", Value(0.7), "ratio (<1.0) between the maximum linear speed and the maximum angular speed.").asDouble();
    robot_type_s = general_options.check("robot_type", Value("none"), "geometry of the robot").asString();
    useRos = general_options.check("use_ROS", Value(false), "enable ROS communications").asBool();

    double tmp = 0;
    tmp = (general_options.check("max_angular_vel", Value(0), "maximum angular velocity of the platform [deg/s]")).asDouble();
    if (tmp >= 0) { max_angular_vel = tmp; }
    else { yError() << "Invalid max_angular_vel"; return false; }
    tmp = (general_options.check("max_linear_vel", Value(0), "maximum linear velocity of the platform [m/s]")).asDouble();
    if (tmp >= 0) { max_linear_vel = tmp; }
    else { yError() << "Invalid max_linear_vel"; return false; }
    tmp = (general_options.check("max_angular_acc", Value(0), "maximum angular acceleration of the platform [deg/s]")).asDouble();
    if (tmp >= 0) { max_angular_acc = tmp; }
    else { yError() << "Invalid max_angular_acc"; return false; }
    tmp = (general_options.check("max_linear_acc", Value(0), "maximum linear acceleration of the platform [m/s]")).asDouble();
    if (tmp >= 0) { max_linear_acc = tmp; }
    else { yError() << "Invalid max_linear_acc"; return false; }

    // open the control board driver
    yInfo("Opening the motors interface...\n");

    int trials = 0;
    double      start_time = yarp::os::Time::now();
    Property    control_board_options("(device remote_controlboard)");

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

    //initialize ROS
    if (useRos)
    {
        if (ctrl_options.check("ROS_GENERAL"))
        {
            string rosNodeName;
            yarp::os::Bottle r_group = ctrl_options.findGroup("ROS_GENERAL");
            if (r_group.check("node_name") == false)
            {
                yError() << "Missing node_name parameter"; return false;
            }
            rosNodeName = r_group.find("node_name").asString();
            rosNode = new yarp::os::Node(rosNodeName);
        }
        else
        {
            yError() << "[ROS_GENERAL] group is missing from configuration file. ROS communication will not be initialized";
        }
    }

    //create the odometry and the motor handlers
    odometry_enabled = true;
    odometry_aux_enabled = false;
    if (rf.check("no_odometry"))
    {
        odometry_enabled = false;
    }
    if (rf.check("enable_aux_odometry"))
    {
        odometry_aux_enabled = true;
    }

    if (robot_type_s == "cer")
    {
        yInfo("Using cer robot type");
        robot_type = ROBOT_TYPE_DIFFERENTIAL;
        if (odometry_enabled)
        {
            odometry_handler = new CER_Odometry(control_board_driver);
            if (odometry_aux_enabled)
            {
                odometry_handler_aux = new CER_Odometry(control_board_driver);
            }
        }
        motor_handler = new CER_MotorControl(control_board_driver);
        input_handler = new Input();
        yarp::os::Property& robot_geom = ctrl_options.addGroup("ROBOT_GEOMETRY");
        robot_geom.put("geom_r", 320.0 / 2 / 1000.0);
        robot_geom.put("geom_L", 338 / 1000.0);
    }
    else if (robot_type_s == "ikart_V1")
    {
        yInfo("Using ikart_V1 robot type");
        robot_type = ROBOT_TYPE_THREE_ROTOCASTER;
        if (odometry_enabled)
        {
            odometry_handler = new iKart_Odometry(control_board_driver);
            if (odometry_aux_enabled)
            {
                odometry_handler_aux = new CER_Odometry(control_board_driver);
            }
        }
        motor_handler = new iKart_MotorControl(control_board_driver);
        input_handler = new Input();
        yarp::os::Property& robot_geom = ctrl_options.addGroup("ROBOT_GEOMETRY");
        robot_geom.put("geom_r", 62.5 / 1000.0);
        robot_geom.put("geom_L", 273 / 1000.0);
        robot_geom.put("g_angle", 0.0);
    }
    else if (robot_type_s == "ikart_V2")
    {
        yInfo("Using ikart_V2 robot type");
        robot_type = ROBOT_TYPE_THREE_MECHANUM;
        if (odometry_enabled)
        {
            odometry_handler = new iKart_Odometry(control_board_driver);
            if (odometry_aux_enabled)
            {
                odometry_handler_aux = new CER_Odometry(control_board_driver);
            }
        }
        motor_handler = new iKart_MotorControl(control_board_driver);
        input_handler = new Input();
        yarp::os::Property& robot_geom = ctrl_options.addGroup("ROBOT_GEOMETRY");
        robot_geom.put("geom_r", 76.15 / 1000.0);
        robot_geom.put("geom_L", 273 / 1000.0);
        robot_geom.put("g_angle", 45.0);
    }
    else
    {
        yError() << "Invalid Robot type selected: ROBOT_TYPE_NONE";
        return false;
    }

    if (odometry_handler && odometry_handler->open(ctrl_options) == false)
    {
        yError() << "Problem occurred while opening odometry handler";
        return false;
    }

    if (odometry_handler_aux)
    {
        Property aux_options = ctrl_options;
        yarp::os::Bottle& robot_geom = aux_options.findGroup("ROBOT_GEOMETRY");
        string s1 = aux_options.toString();
        Value& geom_r = robot_geom.find("geom_r");
        Value& geom_L = robot_geom.find("geom_L");
        if (ctrl_options.check("ODOMETRY_SIMULATED_ERROR") == false)
        {
            yError() << "Requested odometry aux with parameter --enable_aux_odometry, but group [ODOMETRY_SIMULATED_ERROR] was not found.";
            return false;
        }
        yarp::os::Bottle& odometry_error = ctrl_options.findGroup("ODOMETRY_SIMULATED_ERROR");
        if (ctrl_options.check("c1_r") == false) { yError() << "Missing param ODOMETRY_SIMULATED_ERROR::c1_r"; }
        if (ctrl_options.check("c2_L") == false) { yError() << "Missing param ODOMETRY_SIMULATED_ERROR::c2_L"; }
        double c1 = odometry_error.find("c1_r").asDouble();
        double c2 = odometry_error.find("c2_L").asDouble();
        geom_r = Value(geom_r.asDouble() * c1);
        geom_L = Value(geom_L.asDouble() * c2);
        string s2 = aux_options.toString();
        aux_options.put("odometry_portname_suffix", "/aux/odometry_aux:o");
        aux_options.put("odometer_portname_suffix", "/aux/odometer_aux:o");
        aux_options.put("velocity_portname_suffix", "/aux/velocity_aux:o");
        aux_options.unput("ROS_GENERAL");
        aux_options.unput("ROS_ODOMETRY");
        aux_options.unput("ROS_FOOTPRINT");
        aux_options.unput("ROS_INPUT");
        Value& use_ros = aux_options.findGroup("GENERAL").find("use_ROS");
        use_ros = Value(false);
        if (odometry_handler_aux->open(aux_options) == false)
        {
            yError() << "Problem occurred while opening odometry aux handler";
            return false;
        }
    }

    if (motor_handler->open(ctrl_options) == false)
    {
        yError() << "Problem occurred while opening motor handler";
        return false;
    }

    if (input_handler->open(ctrl_options) == false)
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

    kp[0]             = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("kp", Value(0), "kp gain").asDouble();
    kd[0]             = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("kd", Value(0), "kd gain").asDouble();
    ki[0]             = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("ki", Value(0), "ki gain").asDouble();
    sat[0](0, 0)      = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("min", Value(0), "min").asDouble();
    sat[0](0, 1)      = ctrl_options.findGroup("LINEAR_VELOCITY_PID").check("max", Value(0), "max").asDouble();
                      
    kp[1]             = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("kp", Value(0), "kp gain").asDouble();
    kd[1]             = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("kd", Value(0), "kd gain").asDouble();
    ki[1]             = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("ki", Value(0), "ki gain").asDouble();
    sat[1](0, 0)      = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("min", Value(0), "min").asDouble();
    sat[1](0, 1)      = ctrl_options.findGroup("ANGULAR_VELOCITY_PID").check("max", Value(0), "max").asDouble();

    linear_speed_pid  = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[0], ki[0], kd[0], wp[0], wi[0], wd[0], N[0], Tt[0], sat[0]);
    angular_speed_pid = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[1], ki[1], kd[1], wp[1], wi[1], wd[1], N[1], Tt[1], sat[1]);

    linear_ol_pid     = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[0], ki[0], kd[0], wp[0], wi[0], wd[0], N[0], Tt[0], sat[0]);
    angular_ol_pid    = new iCub::ctrl::parallelPID(thread_period / 1000.0, kp[1], ki[1], kd[1], wp[1], wi[1], wd[1], N[1], Tt[1], sat[1]);

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
