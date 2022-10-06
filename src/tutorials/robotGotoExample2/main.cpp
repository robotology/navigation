/*
* Copyright (C)2022 - Istituto Italiano di Tecnologia
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

YARP_LOG_COMPONENT(ROBOTGOTO_EXAMPLE, "navigation.robotGotoExample2")

class robotGotoExample2
{
    private:
    PolyDriver ddNavClient;
    INavigation2D* iNav = nullptr;
    string m_nameof_remote_navigation_port;
    string m_nameof_remote_map_port;
    string m_nameof_remote_localization_port;
    Map2DLocation pos;
    Map2DLocation curr_goal;
        
    public:
    robotGotoExample2() {}
    
    bool open()
    {
        //opens a navigation2DClient device to control the robot
        Property        navTestCfg;
        navTestCfg.put("device", NAVIGATION_CLIENT_DEVICE_DEFAULT);
        navTestCfg.put("local", "/robotGotoExample2");
        navTestCfg.put("navigation_server", m_nameof_remote_navigation_port);
        navTestCfg.put("map_locations_server", m_nameof_remote_map_port);
        navTestCfg.put("localization_server", m_nameof_remote_localization_port);
        bool ok = ddNavClient.open(navTestCfg);
        if (!ok)
        {
            yCError(ROBOTGOTO_EXAMPLE) << "Unable to open navigation2DClient device driver";
            return false;
        }
        
        //gets a INavigation2D interface from the navigation2DClient
        INavigation2D* iNav = 0;
        ok = ddNavClient.view(iNav);
        if (!ok)
        {
            yCError(ROBOTGOTO_EXAMPLE) << "Unable to open INavigation2D interface";
            return false;
        }
        yarp::os::Time::delay(0.1);

        //interrupts any previous navigation task
        iNav->stopNavigation();

        return true;
    }

    bool close()
    {
        //Closes the navigation2DClient
        yCInfo(ROBOTGOTO_EXAMPLE) << "RobotGotoExample2 complete";
        yarp::os::Time::delay(1.0);
        ddNavClient.close();
        return true;
    }

    void wait_idle()
    {
        //Gets the current navigation status
        NavigationStatusEnum status;
        do
        {
            iNav->getNavigationStatus(status);
            if (status == navigation_status_idle) break;

            yCInfo(ROBOTGOTO_EXAMPLE) << "Current navigation status:" << yarp::os::Vocab32::decode(status) << "Waiting for navigation_status_idle";
            yarp::os::Time::delay(0.1);
        } while (1);
    }
    
    void set_goal(string loc_name)
    {
        iNav->gotoTargetByLocationName(loc_name);
    }

    void set_goal(Map2DLocation goal)
    {
        iNav->gotoTargetByAbsoluteLocation(goal);
    }
        
    void wait_complete()
    {  
        NavigationStatusEnum status;
                
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
            yCInfo(ROBOTGOTO_EXAMPLE) << "Current goal:" << curr_goal.toString();

            if (curr_goal.map_id == pos.map_id)
            {
                yCInfo(ROBOTGOTO_EXAMPLE) << "Distance from goal:" << sqrt(pow((pos.x - curr_goal.x), 2) + pow((pos.y - curr_goal.y), 2));
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
        
        iNav->stopNavigation();
    }
};

int main(int argc, char* argv[])
{
    //reads configuration parameters from .ini file
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc, argv);
  
    if (rf.check("help"))
    {
        yCInfo(ROBOTGOTO_EXAMPLE,"Syntax:");
        yCInfo(ROBOTGOTO_EXAMPLE,"robotGotoExamlple2 --gotoLoc <location_name> --navServer_name /nav_server");
        yCInfo(ROBOTGOTO_EXAMPLE,"robotGotoExamlple2 --gotoAbs (<map_name> <x> <y> <t>) --navServer_name /nav_server");
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

    //choose a type of navigation
    if (rf.check("gotoLoc"))
    {
        string loc_name = rf.find("gotoLoc").asString();
        robotGotoExample2 client;
        if (!client.open()) {return -1;}
        client.wait_idle();
        client.set_goal(loc_name);
        client.wait_complete();
        client.close();
    }
    else if (rf.check("gotoAbs"))
    {
        yarp::os::Bottle* goal_bot = rf.find("gotoAbs").asList();
        robotGotoExample2 client;
        if (!client.open()) {return -1;}
        Map2DLocation goal;
        goal.map_id = goal_bot->get(0).asString();
        goal.x = goal_bot->get(1).asFloat32();
        goal.y = goal_bot->get(2).asFloat32();
        goal.theta = goal_bot->get(3).asFloat32();
        client.wait_idle();
        client.set_goal(goal);
        client.wait_complete();
        client.close();
    }
    else
    {
        yCError(ROBOTGOTO_EXAMPLE) << "Unable to open navigation2DClient device driver";
        return -1;
    }
    
    return 0;
}
