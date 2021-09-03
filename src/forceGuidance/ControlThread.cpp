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
 
 This module implements an admittance control which allows to an iKart platform to react to external forces
applied to iCub F/T sensors.
***WARNING***: This module has been imported from legacy iKart repository.
It is untested and should be considered experimental/under development.
 
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

#include "ControlThread.h"

YARP_LOG_COMPONENT(FORCE_GUID, "navigation.forceGuidance")

CtrlThread::CtrlThread(unsigned int _period, ResourceFinder &_rf) :  PeriodicThread(_period), rf(_rf)
{
    control_board_driver=0;
    iVel=0;
}


bool CtrlThread::threadInit()
{
    //opens the ports to receive torques from WBD
    l_wrench_in_port.open ("/forceGuidance/l_wrenches:i");
    r_wrench_in_port.open ("/forceGuidance/r_wrenches:i");
    //opens port to send velocity command to baseControl
    commands_out_port.open("/forceGuidance/commands:o");
    //performs automatic connections
    Network::connect("/wholeBodyDynamics/left_arm/cartesianEndEffectorWrench:o","/forceGuidance/l_wrenches:i");
    Network::connect("/wholeBodyDynamics/right_arm/cartesianEndEffectorWrench:o","/forceGuidance/r_wrenches:i");

    //read initial wrenches from WBD.
    yarp::sig::Vector* l_wrench = l_wrench_in_port.read(true);
    yarp::sig::Vector* r_wrench = r_wrench_in_port.read(true);

    //These offsets will be subtracted in main the control loop.
    lx0=l_wrench->data()[0];
    ly0=l_wrench->data()[1];
    rx0=r_wrench->data()[0];
    ry0=r_wrench->data()[1];
    return true;
}

void CtrlThread::afterStart(bool s)
{
    if (s)
        yCInfo(FORCE_GUID,"Thread started successfully");
    else
        yCError(FORCE_GUID,"Thread did not start");
}

double CtrlThread::lp_filter_1Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 1Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /1.689454484e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
    return yv[1][i];
}

double CtrlThread::lp_filter_0_5Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 0.5Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /3.282051595e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.9390625058 * yv[0][i]);
    return yv[1][i];
}

void CtrlThread::run()
{
    //read the current wrenches on the end effector of the robot
    yarp::sig::Vector* l_wrench = l_wrench_in_port.read(false);
    yarp::sig::Vector* r_wrench = r_wrench_in_port.read(false);
        
    //removes wrenches offset
    if (l_wrench!=0)
    {
        ly=-(l_wrench->data()[0]-lx0);
        lx=+(l_wrench->data()[1]-ly0);
    }

     //removes wrenches offset
    if (r_wrench!=0)
    {
        ry=-(r_wrench->data()[0]-rx0);
        rx=+(r_wrench->data()[1]-ry0);
    }

    //perform filtering
    lx = lp_filter_0_5Hz(lx,0);
    ly = lp_filter_0_5Hz(ly,1);
    rx = lp_filter_0_5Hz(rx,2);
    ry = lp_filter_0_5Hz(ry,3);

    //use a simple admittance control to computes Cartesian velocities
    double linear_gain = 60;   // 60N  = 100%
    double angular_gain = 60;  // 60Nm = 100%
    double desired_direction = atan2( lx+rx, ly+ry ) * 180.0 / 3.14159265;
    double linear_speed      = sqrt ( pow(lx+rx,2)+ pow(ly+ry,2) ) / linear_gain * 100;
    double angular_speed     = (ly-ry) / angular_gain * 100;
    const double pwm_gain = 100;

    //dead band
    if (fabs(angular_speed)<10) angular_speed = 0;
    if (fabs(linear_speed) <10) linear_speed = 0;

    //control saturation
    linear_speed=  (linear_speed > +100) ? +100:linear_speed;
    linear_speed=  (linear_speed < -100) ? -100:linear_speed;
    angular_speed= (angular_speed > +100) ? +100:angular_speed;
    angular_speed= (angular_speed < -100) ? -100:angular_speed;
        
    yCDebug (FORCE_GUID,"(%+8.2f %+8.2f)(%+8.2f %+8.2f)      %+9.1f %+9.1f %+9.1f %+8.0f\n",lx,ly,rx,ry,desired_direction,linear_speed,angular_speed,pwm_gain);

    //send data to yarp output port (to baseControl)
    Bottle bot; 
    bot.addInt32(1);
    bot.addFloat64(desired_direction);
    bot.addFloat64(linear_speed);
    bot.addFloat64(angular_speed);
    bot.addFloat64(pwm_gain);
    commands_out_port.write(bot);
}

void CtrlThread::threadRelease()
{
    //ports cleanup
    commands_out_port.interrupt();
    commands_out_port.close();
    l_wrench_in_port.interrupt();
    l_wrench_in_port.close();
    r_wrench_in_port.interrupt();
    r_wrench_in_port.close();
}
