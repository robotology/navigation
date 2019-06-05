/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <iostream>
#include <iomanip>
#include <string>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

int main(int argc, char* argv[])
{
    //initializes YARP and the resource finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("navigation2DClientTest");
    rf.setDefaultConfigFile("navigation2DClientTest.ini");
    rf.configure(argc, argv);

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

    bool ok;
    Property navTestCfg;
    yarp::dev::PolyDriver      ddNavClient;
    yarp::dev::INavigation2D*  iNav = nullptr;
    navTestCfg.put("device", "navigation2DClient");
    navTestCfg.put("local", "/navigationTest");
    navTestCfg.put("navigation_server", "/navigationServer");
    navTestCfg.put("map_locations_server", "/mapServer");
    navTestCfg.put("localization_server", "/localizationServer");

    ok = ddNavClient.open(navTestCfg);
    if (!ok) { yFatal(); }
    ok = ddNavClient.view(iNav);
    if (!ok) { yFatal(); }

    double timeout = 1.0; //s
    ok = iNav->applyVelocityCommand(1.0, 0.0, 0.0, timeout);
    yarp::os::Time::delay(3);
    ok = iNav->applyVelocityCommand(-1.0, 0.0, 0.0, timeout);
    yarp::os::Time::delay(3);
    ok = iNav->applyVelocityCommand(0.0, 0.0, 90.0, timeout);
    yarp::os::Time::delay(3);
    ok = iNav->applyVelocityCommand(0.0, 0.0, -90.0, timeout);
    yarp::os::Time::delay(3);

    ddNavClient.close();
}
