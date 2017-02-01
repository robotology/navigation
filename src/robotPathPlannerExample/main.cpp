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
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>

#include <iostream>
#include <iomanip>
#include <string>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

void gotoLoc(string location, INavigation2D* iNav)
{
    NavigationStatusEnum status;
    do
    {
        iNav->getNavigationStatus(status);
        if (status == navigation_status_idle) break;

        yInfo() << "Current navigation status:" << status << "Waiting for navigation_status_idle";
        yarp::os::Time::delay(0.1);
    } while (1);

    Map2DLocation pos;
    Map2DLocation current_waypoint;
    iNav->gotoTargetByLocationName(location);

    do
    {
        iNav->getNavigationStatus(status);
        iNav->getCurrentPosition(pos);
        if (status == navigation_status_goal_reached) break;
        if (status == navigation_status_aborted) break;

        yInfo() << "Current navigation status:" << status;
        yInfo() << "Current position:" << pos.toString();
        yInfo() << "Current goal:" << current_waypoint.toString();

        if (current_waypoint.map_id == pos.map_id)
        {
            yInfo() << "Distance from goal:" << sqrt(pow((pos.x - current_waypoint.x), 2) + pow((pos.y - current_waypoint.y), 2));
        }
        else
        {
            yInfo() << "Your current position is not in the same map of the current goal.";
            iNav->stopNavigation();
        }

        yarp::os::Time::delay(1.0);
    } while (1);

    if (status == navigation_status_goal_reached)
    {
        yInfo() << "goal reached!";
    }
    else if (status == navigation_status_aborted)
    {
        yInfo() << "Unable to reach goal";
    }

    iNav->stopNavigation();
}

int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("robotPathPlannerExample");
    rf.setDefaultConfigFile("robotPathPlannerExample.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo("No help message");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    Property        navTestCfg;
    navTestCfg.put("device", "navigation2DClient");
    navTestCfg.put("local", "/robotPathPlannerExample");
    navTestCfg.put("navigation_server", "/robotPathPlanner");
    navTestCfg.put("locations_server", "/locationsServer");
    navTestCfg.put("localization_server", "/localizationServer");

    PolyDriver ddNavClient;
    bool ok = ddNavClient.open(navTestCfg);
    if (!ok)
    {
        yError() << "Unable to open navigation2DClient device driver";
    }

    INavigation2D* iNav = 0;
    ok = ddNavClient.view(iNav);
    if (!ok)
    {
        yError() << "Unable to open INavigation2D interface";
    }

    for (int i = 0; i < 5; i++)
    {
        gotoLoc("living room", iNav);
        gotoLoc("office", iNav);
    } 

    yarp::os::Time::delay(1.0);
    ddNavClient.close();
    return 0;
}
