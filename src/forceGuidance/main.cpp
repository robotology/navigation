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
#include "ControlThread.h"
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

YARP_LOG_COMPONENT(FORCE_GUID_M, "navigation.forceGuidance.main")

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

        // get params from the RF
        ctrlName=rf.check("local",Value("forceGuidance")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();
        partName = rf.check("part", Value("wheels")).asString();

        remoteName = slash + robotName + slash + partName;
        localName= slash + ctrlName;

        thr=new CtrlThread(20,rf);
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


//////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yCInfo(FORCE_GUID_M) << "Options:";
        yCInfo(FORCE_GUID_M) << "\tNo options at the moment" ;
        return 0;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}



