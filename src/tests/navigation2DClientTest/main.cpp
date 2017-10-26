/*
* Copyright (C)2016  iCub Facility - Istituto Italiano di Tecnologia
* Author: Andrea Ruzzenenti
* email:  andrea.ruzzenenti@iit.it
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
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "navigation2DClientTestModule.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("navigation2DClienTest");
    rf.setDefaultConfigFile("navigation2DClientTest.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("No help message yet.. ");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    NavTestModule mod;

    return mod.runModule(rf);
}
