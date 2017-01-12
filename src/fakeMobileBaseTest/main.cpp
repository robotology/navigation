/*
* Copyright (C)2017  iCub Facility - Istituto Italiano di Tecnologia
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

#include "input.h"
#include "controller.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    Input*          input;
    Controller*     control;
    Port            rpcPort;

public:
    CtrlModule() 
    {
        input=0;
        control = 0;
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
        ctrlName=rf.check("local",Value("fakeMobileBaseTest")).asString();
        robotName=rf.check("robot",Value("fakeRobot")).asString();
        partName = rf.check("part", Value("mobile_base")).asString();

        remoteName=slash+robotName+slash+partName;
        localName=slash+ctrlName;
        
        //reads the configuration file
        Property ctrl_options;

        ConstString configFile=rf.findFile("from");
        if (configFile=="") //--from baseCtrl.ini
        {
            yError("Cannot find .ini configuration file. By default I'm searching for fakeMobileBaseTest.ini");
          //  return false;
        }
        else
        {
            ctrl_options.fromConfigFile(configFile.c_str());
        }

        ctrl_options.put("remote", remoteName.c_str());
        ctrl_options.put("local", localName.c_str());

        int rc_count =0;
        int rp_count =0;
        int rf_count =0;
        double start_time=yarp::os::Time::now();
        bool not_yet_connected=true;

        //set the thread rate
        int period = rf.check("period",Value(20)).asInt();
        yInfo("baseCtrl thread rate: %d ms.",period);
        input = new Input();
        if (input->open(rf, ctrl_options) ==false)
        {
            yError() << "Module failed to launch";
            return false;
        }

        control = new Controller();
        if (control->init() == false)
        {
            yError() << "Module failed to launch";
            return false;
        }

        //try to connect to joystickCtrl output
        if (rf.check("joystick_connect"))
        {
            int joystick_trials = 0; 
            do
            {
                yarp::os::Time::delay(1.0);
                if (yarp::os::Network::connect("/joystickCtrl:o", localName + "/joystick:i"))
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
            reply.addString("reset_odometry");
            return true;
        }
        else if (command.get(0).asString()=="reset_odometry")
        {
            if (control)
            {
                control->reset();
                reply.addString("Odometry reset done.");
            }
            return true;
        }
        reply.addString("Unknown command.");
        return true;
    }

    virtual bool close()
    {
        if (input)
        {
            delete input;
           input = 0;
        }
        if (control)
        {
            delete control;
            control = 0;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()
    {
        return 0.01;
    }

    virtual bool   updateModule()
    { 
        double linear_speed = 0;
        double angular_speed = 0;
        double desired_dir = 0;
        double pwm = 0;
        input->read_inputs(&linear_speed, &angular_speed, &desired_dir, &pwm);
        control->apply_control(linear_speed, angular_speed, desired_dir, pwm);
        control->publish();

        static int life_counter=0;
        if (life_counter % 100 == 0)
        {
            input->printStats();
            yInfo("* Life: %d\n\n", life_counter);
        }
        life_counter++;

        return true;
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("navigation");
    rf.setDefaultConfigFile("fakeMobileBaseTest.ini");
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
