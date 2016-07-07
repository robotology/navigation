/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

/** 
\defgroup foceGuidance forceGuidance
 
Force control for iKart platform.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
@@@TODO
 
\section portsa_sec Ports Accessed
 
@@@TODO
 
\section portsc_sec Ports Created 
 
@@@TODO

\section in_files_sec Input Data Files

@@@TODO

\section out_data_sec Output Data Files 

@@@TODO
 
\section conf_file_sec Configuration Files

@@@TODO

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <cstring>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class CtrlThread: public RateThread
{

private:         
    Network             yarp;

protected:
    ResourceFinder      &rf;
    PolyDriver          *control_board_driver;
    IVelocityControl    *iVel;
    double              lx0;
    double              ly0;
    double              rx0;
    double              ry0;
    double              lx;
    double              ly;
    double              rx;
    double              ry;

    Port                            commands_out_port;
    BufferedPort<yarp::sig::Vector> l_wrench_in_port;
    BufferedPort<yarp::sig::Vector> r_wrench_in_port;

    string remoteName;
    string localName;

public:
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) 
    {
    }


    virtual bool threadInit()
    {
        l_wrench_in_port.open ("/forceGuidance/l_wrenches:i");
        r_wrench_in_port.open ("/forceGuidance/r_wrenches:i");
        commands_out_port.open("/forceGuidance/commands:o");
        Network::connect("/wholeBodyDynamics/left_arm/cartesianEndEffectorWrench:o","/forceGuidance/l_wrenches:i");
        Network::connect("/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o","/forceGuidance/r_wrenches:i");

        yarp::sig::Vector* l_wrench = l_wrench_in_port.read(true);
        yarp::sig::Vector* r_wrench = r_wrench_in_port.read(true);

        lx0=l_wrench->data()[0];
        ly0=l_wrench->data()[1];
        rx0=r_wrench->data()[0];
        ry0=r_wrench->data()[1];
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            yInfo("Thread started successfully");
        else
            yError("Thread did not start");
    }

    double lp_filter_1Hz(double input, int i)
    {
       //This is a butterworth low pass first order, with a cut off freqency of 1Hz
       //It must be used with a sampling frequency of 50Hz (20ms)
       static double xv[2][10], yv[2][10];
       xv[0][i] = xv[1][i]; 
       xv[1][i] = input /1.689454484e+01;
       yv[0][i] = yv[1][i]; 
       yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
       return yv[1][i];
    }
    double lp_filter_0_5Hz(double input, int i)
    {
       //This is a butterworth low pass first order, with a cut off freqency of 0.5Hz
       //It must be used with a sampling frequency of 50Hz (20ms)
       static double xv[2][10], yv[2][10];
       xv[0][i] = xv[1][i]; 
       xv[1][i] = input /3.282051595e+01;
       yv[0][i] = yv[1][i]; 
       yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.9390625058 * yv[0][i]);
       return yv[1][i];
    }

    virtual void run()
    {
        yarp::sig::Vector* l_wrench = l_wrench_in_port.read(false);
        yarp::sig::Vector* r_wrench = r_wrench_in_port.read(false);
        
        if (l_wrench!=0)
        {
            ly=-(l_wrench->data()[0]-lx0);
            lx=+(l_wrench->data()[1]-ly0);
        }

        if (r_wrench!=0)
        {
            ry=-(r_wrench->data()[0]-rx0);
            rx=+(r_wrench->data()[1]-ry0);
        }

        lx = lp_filter_0_5Hz(lx,0);
        ly = lp_filter_0_5Hz(ly,1);
        rx = lp_filter_0_5Hz(rx,2);
        ry = lp_filter_0_5Hz(ry,3);

        double linear_gain = 60;   // 60N  = 100%
        double angular_gain = 60;  // 60Nm = 100%
        double desired_direction = atan2( lx+rx, ly+ry ) * 180.0 / 3.14159265;
        double linear_speed      = sqrt ( pow(lx+rx,2)+ pow(ly+ry,2) ) / linear_gain * 100;
        double angular_speed     = (ly-ry) / angular_gain * 100;
        const double pwm_gain = 100;

        //dead band
        if (fabs(angular_speed)<10) angular_speed = 0;
        if (fabs(linear_speed) <10) linear_speed = 0;

        //saturation
        linear_speed=  (linear_speed > +100) ? +100:linear_speed;
        linear_speed=  (linear_speed < -100) ? -100:linear_speed;
        angular_speed= (angular_speed > +100) ? +100:angular_speed;
        angular_speed= (angular_speed < -100) ? -100:angular_speed;
        
        yDebug ("(%+8.2f %+8.2f)(%+8.2f %+8.2f)      %+9.1f %+9.1f %+9.1f %+8.0f\n",lx,ly,rx,ry,desired_direction,linear_speed,angular_speed,pwm_gain);


        //send data to yarp output port
        Bottle bot; 
        bot.addInt(1);
        bot.addDouble(desired_direction);
        bot.addDouble(linear_speed);
        bot.addDouble(angular_speed);
        bot.addDouble(pwm_gain);
        commands_out_port.write(bot);

    }

    virtual void threadRelease()
    {    
        commands_out_port.interrupt();
        commands_out_port.close();
        l_wrench_in_port.interrupt();
        l_wrench_in_port.close();
        r_wrench_in_port.interrupt();
        r_wrench_in_port.close();
    }

};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    //Port        rpcPort;

public:
    CtrlModule() { }

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
        ctrlName=rf.check("local",Value("forceGuidance")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();
        partName = rf.check("part", Value("wheels")).asString();

        remoteName = slash + robotName + slash + partName;
        localName= slash + ctrlName;

        thr=new CtrlThread(20,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        //rpcPort.open((localName+"/rpc").c_str());
        //attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;
        return true;
    }

    virtual double getPeriod()    { return 10.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:";
        yInfo() << "\tNo options at the moment" ;
        return 0;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}



