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
#include <yarp/os/PeriodicThread.h>
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

class CtrlThread: public PeriodicThread
{
private:
    Network             yarp;

protected:
    ResourceFinder      &rf;
    
    //remotoreControlBoard device driver
    PolyDriver          *control_board_driver;
    IVelocityControl    *iVel;

    //wrench offset
    double              lx0;
    double              ly0;
    double              rx0;
    double              ry0;
    //current measurements
    double              lx;
    double              ly;
    double              rx;
    double              ry;

    //yarp in/out ports
    Port                            commands_out_port;
    BufferedPort<yarp::sig::Vector> l_wrench_in_port;
    BufferedPort<yarp::sig::Vector> r_wrench_in_port;

public:
    /**
    * Constructor.
    * @param _period the control loop period (default 20ms)
    * @param _rf the resource finder containing the configuration options
    */
    CtrlThread(unsigned int _period, ResourceFinder &_rf);

    //Inherited from yarp::os::RateThread 
    virtual bool threadInit();
    virtual void afterStart(bool s);
    virtual void run();
    virtual void threadRelease();

    /**
    * These functions implement a butterworth low pass first order, with a cut off frequency of 0.5-1Hz.
    * Coefficients have been computed in order to be used with a sampling frequency of 50Hz (20ms)
    * @param input the value to be filtered
    * @param i the id(0-9) containing the variable history
    * @return the filtered value
    */
    double lp_filter_1Hz(double input, int i);
    double lp_filter_0_5Hz(double input, int i);
};
