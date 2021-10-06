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

#include "navigation_defines.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

YARP_LOG_COMPONENT(ROBOTGOTO_EXAMPLE, "navigation.robotGotoExample")

void gotoLoc(Map2DLocation goal, INavigation2D* iNav)
{
    Map2DLocation pos;
    Map2DLocation curr_goal;

    //Gets the current navigation status
    NavigationStatusEnum status;
    do
    {
        iNav->getNavigationStatus(status);
        if (status == navigation_status_idle) break;

        yCInfo(ROBOTGOTO_EXAMPLE) << "Current navigation status:" << yarp::os::Vocab32::decode(status) << "Waiting for navigation_status_idle";
        yarp::os::Time::delay(0.1);
    } while (1);

    //Sends the navigation command
    iNav->gotoTargetByAbsoluteLocation(goal);

    //Checks if navigation is complete
    do
    {
        iNav->getNavigationStatus(status);
        iNav->getCurrentPosition(pos);
        iNav->getAbsoluteLocationOfCurrentTarget(curr_goal);
        if (status == navigation_status_goal_reached) break;
        if (status == navigation_status_aborted) break;
        if (status == navigation_status_failing) break;

        yCInfo(ROBOTGOTO_EXAMPLE) << "Current navigation status:" << yarp::os::Vocab32::decode(status);
        yCInfo(ROBOTGOTO_EXAMPLE) << "Current position:" << pos.toString();
        yCInfo(ROBOTGOTO_EXAMPLE) << "Current goal:" << goal.toString();

        if (goal.map_id == pos.map_id)
        {
            yCInfo(ROBOTGOTO_EXAMPLE) << "Distance from goal:" << sqrt(pow((pos.x - goal.x), 2) + pow((pos.y - goal.y), 2));
        }
        else
        {
            yCError(ROBOTGOTO_EXAMPLE) << "Your current position is not in the same map of the current goal. Unable to reach goal.";
        }
        yarp::os::Time::delay(1.0);
    }
    while (1);

    if (status == navigation_status_goal_reached)
    {
        yCInfo(ROBOTGOTO_EXAMPLE) << "goal reached!";
    }
    else if (status == navigation_status_aborted)
    {
        yCInfo(ROBOTGOTO_EXAMPLE) << "Unable to reach goal: navigation_status_aborted";
    }
    else if (status == navigation_status_failing)
    {
        yCInfo(ROBOTGOTO_EXAMPLE) << "Unable to reach goal: navigation_status_failing";
    }

    //Terminates the navigation task.
    iNav->stopNavigation();
}

int main(int argc, char* argv[])
{
    //reads configuration parameters from .ini file
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("robotPathPlannerExample");
    rf.setDefaultConfigFile("robotPathPlannerExample.ini");
    rf.configure(argc, argv);

    if (rf.check("help"))
    {
        yCInfo(ROBOTGOTO_EXAMPLE,"No help message");
        return 0;
    }

    //initialize yarp
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yCError(ROBOTGOTO_EXAMPLE,"Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    std::string m_nameof_remote_localization_port = LOCALIZATION_REMOTE_PORT_DEFAULT;
    std::string m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    std::string m_nameof_remote_navigation_port = "/robotGoto";
    if (rf.check("navServer_name"))
    {
        m_nameof_remote_navigation_port = rf.find("navServer_name").asString();
    }

    //opens a navigation2DClient device to control the robot
    Property        navTestCfg;
    navTestCfg.put("device", NAVIGATION_CLIENT_DEVICE_DEFAULT);
    navTestCfg.put("local", "/robotGotoExample");
    navTestCfg.put("navigation_server", m_nameof_remote_navigation_port);
    navTestCfg.put("map_locations_server", m_nameof_remote_map_port);
    navTestCfg.put("localization_server", m_nameof_remote_localization_port);
    PolyDriver ddNavClient;
    bool ok = ddNavClient.open(navTestCfg);
    if (!ok)
    {
        yCError(ROBOTGOTO_EXAMPLE) << "Unable to open navigation2DClient device driver";
        return -1;
    }

    //gets a INavigation2D interface from the navigation2DClient
    INavigation2D* iNav = 0;
    ok = ddNavClient.view(iNav);
    if (!ok)
    {
        yCError(ROBOTGOTO_EXAMPLE) << "Unable to open INavigation2D interface";
        return -1;
    }

    //interrupts any previous navigation task
    iNav->stopNavigation();
    yarp::os::Time::delay(0.1);

    //defines two locations
    Map2DLocation goal1;
    Map2DLocation goal2;
    goal1.map_id = "testMap";
    goal1.x = 2.0;
    goal1.y = 1.0;
    goal1.theta = 0;

    goal2.map_id = "testMap";
    goal2.x = 1.0;
    goal2.y = 2.0;
    goal2.theta = 90;

    for (int i = 0; i < 5; i++)
    {
        //move the robot until goal1 location is reached.
        yCInfo(ROBOTGOTO_EXAMPLE) << "Now moving towards location:" << goal1.toString();
        gotoLoc(goal1, iNav);

        //move the robot until goal2 location reached.
        yCInfo(ROBOTGOTO_EXAMPLE) << "Now moving towards location:" << goal2.toString();
        gotoLoc(goal2, iNav);
    }

    //Closes the navigation2DClient
    yCInfo(ROBOTGOTO_EXAMPLE) << "RobotGotoPlannerExample complete";
    yarp::os::Time::delay(1.0);
    ddNavClient.close();
    return 0;
}