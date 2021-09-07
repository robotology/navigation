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
#include <yarp/dev/IMap2D.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <mapUtils.h>

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
    Property mapTestCfg;
    yarp::dev::PolyDriver      ddNavClient;
    yarp::dev::PolyDriver      ddMapClient;
    yarp::dev::Nav2D::INavigation2DTargetActions*  iTrgt  = nullptr;
    yarp::dev::Nav2D::INavigation2DControlActions* iCtrl  = nullptr;
    yarp::dev::Nav2D::INavigation2D*               iNav   = nullptr;
    yarp::dev::Nav2D::INavigation2DExtraActions*   iExtra = nullptr;
    yarp::dev::Nav2D::IMap2D*                      iMap   = nullptr;

    navTestCfg.put("device", "navigation2DClient");
    navTestCfg.put("local", "/navigationTest");
    navTestCfg.put("navigation_server", "/navigationServer");
    navTestCfg.put("map_locations_server", "/mapServer");
    navTestCfg.put("localization_server", "/localizationServer");
    ok = ddNavClient.open(navTestCfg);
    if (!ok) { yFatal(); }

    mapTestCfg.put("device", "map2DClient");
    mapTestCfg.put("local", "/mapTest");
    mapTestCfg.put("navigation_server", "/mapServer");
    ok = ddMapClient.open(mapTestCfg);
    if (!ok) { yFatal(); }


    ok = ddNavClient.view(iTrgt);
    if (!ok) { yFatal(); }
    ok = ddNavClient.view(iCtrl);
    if (!ok) { yFatal(); }
    ok = ddNavClient.view(iNav);
    if (!ok) { yFatal(); }
    ok = ddNavClient.view(iExtra);
    if (!ok) { yFatal(); }

    ok = ddMapClient.view(iMap);
    if (!ok) { yFatal(); }


    double timeout = 1.0; //s
    yarp::dev::Nav2D::Map2DLocation loc;
    yarp::dev::Nav2D::Map2DPath MyPath;
    ok = iNav->gotoTargetByLocationName("goal");
    ok = iCtrl->getAllNavigationWaypoints(yarp::dev::Nav2D::global_trajectory, MyPath);
    ok = iCtrl->stopNavigation();

    std::vector<yarp::dev::Nav2D::Map2DArea> allAreas;
    std::vector<yarp::dev::Nav2D::Map2DArea> areasList;

    iMap->getAllAreas(allAreas);
    areasList = map_utilites::compute_areas_to_cross(MyPath, allAreas);

    auto print = [](const yarp::dev::Nav2D::Map2DArea& n) { std::cout << " " << n.toString(); };

    std::cout << "Input Areas:";
    std::for_each(allAreas.cbegin(), allAreas.cend(), print);
    std::cout << endl;

    std::cout << "Output Areas:";
    std::for_each(areasList.cbegin(), areasList.cend(), print);
    std::cout << endl;

    ddNavClient.close();
}
