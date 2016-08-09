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


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr;
    Port            rpcPort;

public:
    CtrlModule() 
    {
        control_thr=0;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("local",Value("baseControl")).asString();
        robotName=rf.check("robot",Value("cer")).asString();
        partName = rf.check("part", Value("mobile_base")).asString();

        remoteName=slash+robotName+slash+partName;
        localName=slash+ctrlName;
        
        //reads the configuration file
        Property ctrl_options;

        ConstString configFile=rf.findFile("from");
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

        //check for robotInterface availablity
        yInfo("Checking for robotInterface availability");
        Port startport;
        startport.open ("/baseControl/robotInterfaceCheck:rpc");
        

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
            yInfo("skipping robotInterface check");
        }
        else
        {
            do
            {
               if (not_yet_connected)
               {
                  bool rc = yarp::os::Network::connect (localName + "/baseControl/robotInterfaceCheck:rpc","/" + robotName + "/robotInterface");
                  if (rc == false)
                  {
                     yWarning ("Problems trying to connect to RobotInterface %d", rc_count ++);
                     yarp::os::Time::delay (1.0);
                     continue;
                  }
                  else 
                  {
                     not_yet_connected = false;  
                     yDebug ("Connection established with robotInterface");
                  }
               }
    
               bool rp = startport.write (cmd, response);
               if (rp == false)
               {
                  yWarning ("Problems trying to connect to RobotInterface %d", rp_count ++);
                  if (yarp::os::Time::now()-start_time>30)
                  {
                     yError ("Timeout expired while trying to connect to robotInterface");
                     return false;
                  }
                  yarp::os::Time::delay (1.0);
                  continue;
               }
               else 
               {
                  if (response.get(0).asString() != "ok")
                  {
                     yWarning ("RobotInterface is not ready yet, retrying... %d", rf_count++);
                     if (yarp::os::Time::now()-start_time>30)
                     {
                        yError ("Timeout expired while waiting for robotInterface availability");
                        return false;
                     }
                     yarp::os::Time::delay (1.0);
                     continue;
                  }
                  else
                  {
                     yInfo ("RobotInterface is ready");
                     break;
                  }
               }
            } while (1);
        }

        //set the thread rate
        int period = rf.check("period",Value(20)).asInt();
        yInfo("baseCtrl thread rate: %d ms.",period);

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
                if (yarp::os::Network::connect("/joystickCtrl:o","/baseControl/joystick:i"))
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
                if (command.get(1).asInt()>0) 
                    {control_thr->get_motor_handler()->set_motors_filter(command.get(1).asInt()); reply.addString("Motors filter on");}
                else
                    {control_thr->get_motor_handler()->set_motors_filter(0); reply.addString("Motors filter off");}
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
                control_thr->get_odometry_handler()->reset_odometry();
                reply.addString("Odometry reset done.");
            }
            return true;
        }
        reply.addString("Unknown command.");
        return true;
    }

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

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule()
    { 
        if (control_thr)
        {
            control_thr->printStats();
            control_thr->get_motor_handler()->updateControlMode();
            control_thr->get_odometry_handler()->printStats();
            control_thr->get_motor_handler()->printStats();
            control_thr->get_input_handler()->printStats();
        }
        else
        {
            yDebug("* Motor thread:not running");
        }

        static int life_counter=0;
        yInfo( "* Life: %d\n\n", life_counter);
        life_counter++;

        return true;
    }
};



int main(int argc, char *argv[])
{
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

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}
