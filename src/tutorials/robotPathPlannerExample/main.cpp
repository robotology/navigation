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
#include <math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

bool gotoLoc(string location, INavigation2D* iNav)
{
    Map2DLocation pos;
    Map2DLocation current_waypoint;
    Map2DLocation curr_goal;

    //Gets the current navigation status
    NavigationStatusEnum status;
    do
    {
        iNav->getNavigationStatus(status);
        if (status == navigation_status_idle) break;

        yInfo() << "Current navigation status:" << yarp::os::Vocab::decode(status) << "Waiting for navigation_status_idle";
        yarp::os::Time::delay(0.1);
    } while (1);

    iNav->getAbsoluteLocationOfCurrentTarget(curr_goal);
    iNav->getLocation(location,current_waypoint);

    //Sends the navigation command
    if (iNav->gotoTargetByLocationName(location)==false)
    {
        yError() << "Unable to go to: " <<location;
        return false;
    }
    
    //Checks if navigation is complete
    do
    {
        iNav->getNavigationStatus(status);
        iNav->getCurrentPosition(pos);
        if (status == navigation_status_goal_reached) break;
        if (status == navigation_status_aborted) break;
        if (status == navigation_status_failing) break;

        yInfo() << "Current navigation status:" << yarp::os::Vocab::decode(status);
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
    } 
    while (1);

    if (status == navigation_status_goal_reached)
    {
        yInfo() << "goal reached!";
    }
    else if (status == navigation_status_aborted)
    {
        yInfo() << "Unable to reach goal: navigation_status_aborted";
    }
    else if (status == navigation_status_failing)
    {
        yInfo() << "Unable to reach goal: navigation_status_failing";
    }

    //Terminates the navigation task.
    iNav->stopNavigation();
    return true;
}

int main(int argc, char* argv[])
{
    //basic resource finder and yarp initialization
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

    std::string navigation_server_name = "/robotPathPlanner";
    if (rf.check("navServer_name"))
    {
        navigation_server_name = rf.find("navServer_name").asString();
    }

    //opens a navigation2DClient device to control the robot
    Property        navTestCfg;
    navTestCfg.put("device", "navigation2DClient");
    navTestCfg.put("local", "/robotPathPlannerExample");
    navTestCfg.put("navigation_server", navigation_server_name);
    navTestCfg.put("map_locations_server", "/mapServer");
    navTestCfg.put("localization_server", "/localizationServer");
    PolyDriver ddNavClient;
    bool ok = ddNavClient.open(navTestCfg);
    if (!ok)
    {
        yError() << "Unable to open navigation2DClient device driver";
        return -1;
    }

    //gets a INavigation2D interface from the navigation2DClient
    INavigation2D* iNav = 0;
    ok = ddNavClient.view(iNav);
    if (!ok)
    {
        yError() << "Unable to open INavigation2D interface";
        return -1;
    }

    //interrupts any previous navigation task
    iNav->stopNavigation();
    yarp::os::Time::delay(0.1);

    //opens a map2DClient device to communicate with MapServer
    Property map_options;
    PolyDriver ddMapClient;
    map_options.put("device", "map2DClient");
    map_options.put("local", "/robotPathPlannerExample"); //This is just a prefix. map2DClient will complete the port name.
    map_options.put("remote", "/mapServer");
    if (ddMapClient.open(map_options) == false)
    {
        yError() << "Unable to open mapClient";
        return -1;
    }
    yInfo() << "Opened mapClient";

    //gets a IMap2D interface from the map2DClient
    IMap2D*  iMap=0;
    ddMapClient.view(iMap);
    if (ddMapClient.isValid() == false || iMap == 0)
    {
        yError() << "Unable to view map interface";
        return false;
    }

    //defines a new location called bedroom and saves it to map server
    Map2DLocation bedroom_location;
    bedroom_location.map_id="testMap";
    bedroom_location.x=2.0;
    bedroom_location.x=1.0;
    bedroom_location.theta=45.0;
    if (iMap->storeLocation("bedroom",bedroom_location))
    {
        yInfo() << "'bedroom' location has been stored to mapServer";
    }
    else
    {
        yWarning() << "'bedroom' location is already present into mapServer";
    }
    
    for (int i = 0; i < 5; i++)
    {
        //move the robot until 'living_room' location is reached.
        yInfo() << "Now moving towards 'living_room' location";
        gotoLoc("living_room", iNav);

        //move the robot until 'office' location is reached.
        yInfo() << "Now moving towards 'office' location";
        gotoLoc("office", iNav);

        //move the robot until 'bedroom' location is reached.
        yInfo() << "Now moving towards 'bedroom' location";
        gotoLoc("bedroom", iNav);
    } 

    //Closes the navigation2DClient
    yInfo() << "RobotPathPlannerExample complete";
    yarp::os::Time::delay(1.0);
    ddNavClient.close();
    return 0;
}
