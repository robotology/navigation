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
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "controlThread.h"

/**
 * \section baseControl
 * This core module controls the robot joints in order to achieve the desired cartesian velocity command, sent either via a joystick or via YARP port/ROS topic
 * The module requires the definition of the robot kinematics. So far only ikart_V1, ikart_V2, cer types are valid options.as robot_type parameter.
 * Four control modes are available: velocity_no_pid sets the control modes of the motors to velocity and sends individual velocity references, computed by from user cartesian commands, to the joints.
 * velocity_pid also controls the motors in velocity modes, but it also implements and external closed loop on the realized cartesian trajectory.
 * openloop_no_pid/openloop_pid are similar, but the individual motors are controlled by sending pwm references instead of velocity commands.
 * The module also performs the odometry computation and publishes it onto a yarp port/ROS topic.
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr;
    Port            rpcPort;
    bool            verbose_print;

public:
    CtrlModule() 
    {
        control_thr=0;
        verbose_print=true;
    }

    //Module initialization and configuration
    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        // get params from the RF
        ctrlName=rf.check("local",Value("baseControl")).asString();
        robotName=rf.check("robot",Value("cer")).asString();
        partName = rf.check("part", Value("mobile_base")).asString();
        remoteName=slash+robotName+slash+partName;
        localName=slash+ctrlName;
        
        //reads the configuration file
        Property ctrl_options;

        std::string configFile=rf.findFile("from");
        if (configFile=="") //--from baseCtrl.ini
        {
            yError("Cannot find .ini configuration file. By default I'm searching for baseCtrl.ini");
            return false;
        }
        else
        {
            ctrl_options.fromConfigFile(configFile.c_str());
        }

        ctrl_options.put("remote", remoteName.c_str());
        ctrl_options.put("local", localName.c_str());

        //check for robotInterface availability
        yInfo("Checking for yarpRobotInterface availability");
        Port startport;
        startport.open (localName + "/yarpRobotInterfaceCheck:rpc");
        

        Bottle cmd; cmd.addString("is_ready");
        Bottle response;
        int rc_count =0;
        int rp_count =0;
        int rf_count =0;
        double start_time=yarp::os::Time::now();
        bool not_yet_connected=true;

        bool skip_robot_interface_check = rf.check("skip_robot_interface_check");
        if (skip_robot_interface_check)
        {
            yInfo("skipping yarpRobotInterface check");
        }
        else
        {
            do
            {
               if (not_yet_connected)
               {
                  bool rc = yarp::os::Network::connect (localName + "/yarpRobotInterfaceCheck:rpc","/" + robotName + "/yarprobotinterface");
                  if (rc == false)
                  {
                     yWarning ("Problems trying to connect to %s %d", std::string("/" + robotName + "/yarprobotinterface").c_str(), rc_count ++);
                     yarp::os::Time::delay (1.0);
                     continue;
                  }
                  else 
                  {
                     not_yet_connected = false;  
                     yDebug ("Connection established with yarpRobotInterface");
                  }
               }
    
               bool rp = startport.write (cmd, response);
               if (rp == false)
               {
                  yWarning ("Problems trying to connect to yarpRobotInterface %d", rp_count ++);
                  if (yarp::os::Time::now()-start_time>30)
                  {
                     yError ("Timeout expired while trying to connect to yarpRobotInterface");
                     return false;
                  }
                  yarp::os::Time::delay (1.0);
                  continue;
               }
               else 
               {
                  if (response.get(0).asString() != "ok")
                  {
                     yWarning ("yarpRobotInterface is not ready yet, retrying... %d", rf_count++);
                     if (yarp::os::Time::now()-start_time>30)
                     {
                        yError ("Timeout expired while waiting for yarpRobotInterface availability");
                        return false;
                     }
                     yarp::os::Time::delay (1.0);
                     continue;
                  }
                  else
                  {
                     yInfo ("yarpRobotInterface is ready");
                     break;
                  }
               }
            } while (1);
        }

        //set the thread rate
        double period = rf.check("period",Value(0.020)).asDouble();
        yInfo("baseCtrl thread period: %f s.",period);

        //verbosity
        if (rf.check("silent") ||
            rf.check("no_verbose"))
        {
            yInfo("Verbosity off");
            verbose_print=false;
        }

        // the motor control thread
        bool motors_enabled=true;
        if (rf.check("no_motors"))
        {
            yInfo("'no_motors' option found. Skipping motor control part.");
            motors_enabled=false;
        }

        if (motors_enabled==true)
        {
            control_thr = new ControlThread(period, rf, ctrl_options);

            if (!control_thr->start())
            {
                delete control_thr;
                return false;
            }
        }

        //try to connect to joystickCtrl output
        if (rf.check("joystick_connect"))
        {
            int joystick_trials = 0; 
            do
            {
                yarp::os::Time::delay(1.0);
                if (yarp::os::Network::connect("/joystickCtrl:o",localName+"/joystick1:i"))
                    {
                        yInfo("Joystick has been automatically connected");
                        break;
                    }
                else
                    {
                        yWarning("Unable to find the joystick port, retrying (%d/5)...",joystick_trials);
                        joystick_trials++;
                    }

                if (joystick_trials>=5)
                    {
                        yError("Unable to find the joystick port, giving up");
                        break;
                    }
            }
            while (1);
        }

        //check for debug mode
        if (rf.check("debug"))
        {
            this->control_thr->enable_debug(true);
        }

        rpcPort.open((localName+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    //This function parses the user commands received through the RPC port
    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("run");
            reply.addString("idle");
            reply.addString("reset_odometry");
            reply.addString("set_prefilter 0/1/2/4/8");
            reply.addString("set_motors_filter 0/1/2/4/8");
            reply.addString("change_pid <identif> <kp> <ki> <kd>");
            reply.addString("change_ctrl_mode <type_string>");
            reply.addString("set_debug_mode 0/1");
            return true;
        }
        else if (command.get(0).asString()=="set_debug_mode")
        {
            if (control_thr)
            {
                if (command.get(1).asInt()>0)
                    {control_thr->enable_debug(true); reply.addString("debug mode on");}
                else
                    {control_thr->enable_debug(false); reply.addString("debug mode off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="set_prefilter")
        {
            if (control_thr)
            {
                if (command.get(1).asInt()>0) 
                    {control_thr->set_input_filter(command.get(1).asInt()); reply.addString("Prefilter on");}
                else
                    {control_thr->set_input_filter(0); reply.addString("Prefilter off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="set_motors_filter")
        {
            if (control_thr)
            {
                int f= command.get(1).asInt();
                if (f==1) 
                    {control_thr->get_motor_handler()->set_motors_filter(MotorControl::HZ_1); reply.addString("Motors filter on");}
                if (f==2) 
                    {control_thr->get_motor_handler()->set_motors_filter(MotorControl::HZ_2); reply.addString("Motors filter on");}
                if (f==4) 
                    {control_thr->get_motor_handler()->set_motors_filter(MotorControl::HZ_4); reply.addString("Motors filter on");}
                if (f==8) 
                    {control_thr->get_motor_handler()->set_motors_filter(MotorControl::HZ_8); reply.addString("Motors filter on");}
                else
                    {control_thr->get_motor_handler()->set_motors_filter(MotorControl::DISABLED); reply.addString("Motors filter off");}
            }
            return true;
        }
        else if (command.get(0).asString()=="run")
        {
            if (control_thr)
            {
            if      (control_thr->get_control_type() == BASE_CONTROL_NONE)            {control_thr->get_motor_handler()->set_control_idle();}
            else if (control_thr->get_control_type() == BASE_CONTROL_VELOCITY_NO_PID) {control_thr->get_motor_handler()->set_control_velocity();}
            else if (control_thr->get_control_type() == BASE_CONTROL_OPENLOOP_NO_PID) {control_thr->get_motor_handler()->set_control_openloop();}
            else if (control_thr->get_control_type() == BASE_CONTROL_VELOCITY_PID)    {control_thr->get_motor_handler()->set_control_velocity();}
            else if (control_thr->get_control_type() == BASE_CONTROL_OPENLOOP_PID)    {control_thr->get_motor_handler()->set_control_openloop();}

                if (control_thr->get_motor_handler()->check_motors_on())
                    {reply.addString("Motors now on");}
                else
                    {reply.addString("Unable to turn motors on! fault pressed?");}

            }
            return true;
        }
        else if (command.get(0).asString()=="idle")
        {
            if (control_thr)
            {
                control_thr->get_motor_handler()->set_control_idle();
                {reply.addString("Motors now off.");}
            }
            return true;
        }
        else if (command.get(0).asString()=="change_ctrl_mode")
        {
            if (control_thr)
            {
                if (control_thr->set_control_type(command.get(1).asString().c_str()))
                    {reply.addString("control mode changed");}
                else
                    {reply.addString("invalid control mode request");}
            }
            return true;
        }
        else if (command.get(0).asString()=="change_pid")
        {
            if (control_thr)
            {
                string identif = command.get(1).asString().c_str();
                double kp = command.get(2).asDouble();
                double ki = command.get(3).asDouble();
                double kd = command.get(4).asDouble();
                control_thr->set_pid(identif,kp,ki,kd);
                reply.addString("New pid parameters set.");
                yInfo("New pid parameters set.");
            }
            return true;
        }
        else if (command.get(0).asString()=="reset_odometry")
        {
            if (control_thr)
            {
                Odometry* pOdometry=0;
                pOdometry = control_thr->get_odometry_handler();
                if (pOdometry) 
                {pOdometry->reset_odometry(); reply.addString("Odometry reset done.");}
                else
                {reply.addString("No Odometry available.");}
            }
            return true;
        }
        reply.addString("Unknown command.");
        return true;
    }

    //Module close and cleanup
    virtual bool close()
    {
        if (control_thr)
        {
            control_thr->stop();
            delete control_thr;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    //CtrlModule update is called with the frequency of 1s. The update function just checks that everything is ok and
    //prints some stats. The control is performed by the high-frequency thread control_thr.
    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule()
    { 
        if (control_thr)
        {
            Odometry* pOdometry=0;
            if (verbose_print) control_thr->printStats();
            control_thr->get_motor_handler()->updateControlMode();
            pOdometry = control_thr->get_odometry_handler();
            if (pOdometry) { if (verbose_print) pOdometry->printStats(); }
            if (verbose_print) control_thr->get_motor_handler()->printStats();
            if (verbose_print) control_thr->get_input_handler()->printStats();
        }
        else
        {
            yDebug("* Motor thread:not running");
        }

        static int life_counter=0;
        if (verbose_print) yInfo( "* Life: %d\n\n", life_counter);
        life_counter++;

        return true;
    }
};


/////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    //Initialize Yarp network
    Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cerr << "Sorry YARP network does not seem to be available, is the yarp server available?\n";
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("navigation");
    rf.setDefaultConfigFile("baseCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("Possible options: ");
        yInfo("'rate <r>' sets the threads rate (default 20ms).");
        yInfo("'no_filter' disables command filtering.");
        yInfo("'no_motors' motor interface will not be opened.");
        yInfo("'no_start' do not automatically enables pwm.");
        yInfo("'joystick_connect' tries to automatically connect to the joystickCtrl output.");
        yInfo("'skip_robot_interface_check' does not connect to robotInterface/rpc (useful for simulator)");
        return 0;
    }

    //Starts the control module
    CtrlModule mod;
    return mod.runModule(rf);
}
