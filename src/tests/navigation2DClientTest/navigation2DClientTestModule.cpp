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

#include "navigation2DClientTestModule.h"
#include <cmath>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;
#define PERIOD  0
#define TIMEOUT 60

YARP_LOG_COMPONENT(NAV_CLIENT_TEST, "navigation.navigation2DClientTest")

NavTestModule::NavTestModule()
{
    period          = PERIOD;
    locationsStored = false;
    linToll = 0.05;
    angToll = 0.6;
}

NavTestModule::~NavTestModule()
{
    close();
}

double NavTestModule::getPeriod()
{
    return period;
}

bool NavTestModule::close()
{
    return yarp::os::RFModule::close();
}

bool NavTestModule::configure(ResourceFinder& rf)
{
    bool            okClient, okView;
    Property        navTestCfg, pLocationServer_cfg;
    string          navServerRoot;
    Time::now();

    if(rf.check("navServerRoot"))
    {
        navServerRoot = rf.find("navServerRoot").asString();
    }
    else
    {
        navServerRoot = "/robotGoto";
    }
    pLocationServer_cfg.put("device", "locationsServer");
    pLocationServer_cfg.put("local", "/locationServer");
    pLocationServer_cfg.put("ROS_enabled", "");
    bool ok_location = ddLocServer.open(pLocationServer_cfg);
    if(ok_location){yCInfo(NAV_CLIENT_TEST) << "ddLocationServer open reported successful";};

    navTestCfg.put("device",         "navigation2DClient");
    navTestCfg.put("local",          "/navigationTest");
    navTestCfg.put("navigation_server", navServerRoot);
    navTestCfg.put("map_locations_server", "/mapServer");
    navTestCfg.put("localization_server", "/localizationServer");

    iNav            = 0;
    okClient        = ddNavClient.open(navTestCfg);
    okView          = ddNavClient.view(iNav);

    if(okClient)
    {
        yCInfo(NAV_CLIENT_TEST,"navigation2DClient device opened successful;y");
        if(okView)
        {
            yCInfo(NAV_CLIENT_TEST,"INavigation2D interface opened successfully");
        }
        else
        {
            yCError(NAV_CLIENT_TEST,"Error opening INavigation2D interface");
            return false;
        }
    }
    else
    {
        yCError(NAV_CLIENT_TEST,"Error opening navigation2DClient device");
        return false;
    }

    return true;
}

bool NavTestModule::executeStep(navStep s)
{
    size_t               i;
    NavigationStatusEnum status;
    double               time;

    if(!iNav)
    {
        yCError(NAV_CLIENT_TEST,"navigation interface pointer is not valid anymore");
        return false;
    }

    for(i = 0; i < s.frames.size(); i++)
    {
        navFrame& f = s.frames[i];
        yCInfo(NAV_CLIENT_TEST) << "reaching location" << s.label << ". heading towards step n." << i << "located in (relative) x:" << f.x << "y:" << f.y << "th:" << f.t << "for registration purpose";
        iNav->gotoTargetByRelativeLocation(f.x,f.y,f.t);
        iNav->getNavigationStatus(status);
        time = Time::now();
        while (status != navigation_status_goal_reached)
        {
            yarp::os::Time::delay(0.1);
            if(!iNav->getNavigationStatus(status))
            {
                yCError(NAV_CLIENT_TEST,"unable to get navigation status while heading towards frame %lu of step %s", i, s.label.c_str());
                return false;
            }
            if(Time::now() - time >= TIMEOUT)
            {
                yCError(NAV_CLIENT_TEST,"time out while heading towards frame %lu of step %s ", i, s.label.c_str());
                return false;
            }
        }
    }

    if(!iNav->storeCurrentPosition(s.label))
    {
        yCError(NAV_CLIENT_TEST) << "error storing location" << s.label;
    }
    return true;
}

void NavTestModule::printRegisteredLocations()
{
    std::vector<std::string>    locations;
    std::vector<std::string>    emptyLocationList;
    std::vector<Map2DLocation> coords;
    if(!iNav->getLocationsList(locations))
    {
        yCError(NAV_CLIENT_TEST) << "error retrieving location list";
    }

    if(locations.size() == 0)
    {
        yCInfo(NAV_CLIENT_TEST) << "no locations registered";
    }

    for(size_t i = 0; i < locations.size(); i++)
    {
        Map2DLocation l;
        if(!iNav->getLocation(locations[i], l))
        {
            yCError(NAV_CLIENT_TEST) << "error retrieving location" << locations[i];
        }
        else
        {
            coords.push_back(l);
            yCInfo(NAV_CLIENT_TEST) << locations[i] << " is located in x: " << l.x << " y: " << l.y << " and theta: " << l.theta;
        }
    }
    yCInfo(NAV_CLIENT_TEST) << "Test: deleting 1st location";
    if(iNav->deleteLocation(locations[0]))
    {
        yCInfo(NAV_CLIENT_TEST) << "Successful!!";
    }
    else
    {
        yCError(NAV_CLIENT_TEST) << "miserably failed..";
    }

    yCInfo(NAV_CLIENT_TEST) << "Test: deleting every stored location";
    if(iNav->clearAllLocations())
    {
        yCInfo(NAV_CLIENT_TEST) << "Successful!!";
    }
    else
    {
        yCError(NAV_CLIENT_TEST) << "miserably failed..";
    }

    iNav->getLocationsList(emptyLocationList);

    if(emptyLocationList.size())
    {
        yCError(NAV_CLIENT_TEST) << "error.. location server actually contains" << emptyLocationList.size() << "locations";
    }

    yCInfo(NAV_CLIENT_TEST) << "Test: re-storing location removed";
    for(size_t i = 0; i < locations.size(); i++)
    {
        if(!iNav->storeLocation(locations[i], coords[i]))
        {
            yCError(NAV_CLIENT_TEST) << "error re-storing location" << locations[i];
        }
        else
        {
            yCInfo(NAV_CLIENT_TEST) << "Location" << locations[i] << "successfully re-stored";
        }
    }

    for(size_t i = 0; i < locations.size(); i++)
    {
        Map2DLocation l;
        iNav->getLocation(locations[i], l);
        yCDebug(NAV_CLIENT_TEST) << locations[i] << " is located in x: " << l.x << " y: " << l.y << " and theta: " << l.theta;
    }

}

void NavTestModule::absLocationTest()
{
    yCInfo(NAV_CLIENT_TEST) << "Test: go to by absolute location:";
    double time = Time::now();
    if(!iNav->gotoTargetByAbsoluteLocation( Map2DLocation("map",0,0,0) ) )
    {
        yCError(NAV_CLIENT_TEST) << "failed!";
    }
    NavigationStatusEnum status;
    do
    {
        iNav->getNavigationStatus(status);
        yarp::os::Time::delay(0.1);
        if(Time::now() - time >= TIMEOUT)
        {
            yCError(NAV_CLIENT_TEST,"time out!!");
        }
    }
    while(status != navigation_status_goal_reached );
}

void NavTestModule::suspResumeTest()
{
    yCInfo(NAV_CLIENT_TEST) << "Test: suspending navigation";
    if(iNav->suspendNavigation())
    {
        yCInfo(NAV_CLIENT_TEST) << "Successful!!";
    }
    else
    {
        yCError(NAV_CLIENT_TEST) << "miserably failed..";
    }

    yCInfo(NAV_CLIENT_TEST) << "5 second delay..";
    for(size_t i = 0; i < 5; i ++)
    {
        yCDebug(NAV_CLIENT_TEST) << i;
        yarp::os::Time::delay(1);
    }
    yCInfo(NAV_CLIENT_TEST) << "Test: resuming navigation";
    if(iNav->resumeNavigation())
    {
        yCInfo(NAV_CLIENT_TEST) << "Successful!!";
    }
    else
    {
        yCError(NAV_CLIENT_TEST) << "miserably failed..";
    }
}

bool NavTestModule::updateModule()
{
    static size_t        i;
    NavigationStatusEnum status;
    static double        time;

    //storing location
    if(!locationsStored)
    {
        navStep step;
        Map2DLocation initialPos;
        iNav->getCurrentPosition(initialPos);

        //quick version
        /*step.frames.push_back(navFrame(1, 0, 0));
        step.absPos = navFrame(initialPos.x + 1, initialPos.y + 0, initialPos.theta);
        step.label  = "N1";
        stepVector.push_back(step);
        step.frames.clear();

        step.frames.push_back(navFrame(1, 0, 0));
        step.absPos = navFrame(initialPos.x + 2, initialPos.y + 0, initialPos.theta);
        step.label  = "N2";
        stepVector.push_back(step);*/
        //long version
        step.frames.push_back(navFrame(0, -0.5, 0));
        step.frames.push_back(navFrame(0.5, 0, 0));
        step.absPos = navFrame(initialPos.x + 0.5, initialPos.y + -0.5, initialPos.theta);
        step.label  = "NE";
        stepVector.push_back(step);
        step.frames.clear();

        step.frames.push_back(navFrame(0, 1, 90));
        step.absPos = navFrame(initialPos.x + 0.5, initialPos.y + 0.5, initialPos.theta + 90);
        step.label  = "NW";
        stepVector.push_back(step);

        step.absPos = navFrame(initialPos.x - 0.5, initialPos.y + 0.5, -initialPos.theta);
        step.label = "SW";
        stepVector.push_back(step);

        step.absPos = navFrame(initialPos.x - 0.5, initialPos.y - 0.5, initialPos.theta - 90);
        step.label = "SE";
        stepVector.push_back(step);

        for(i = 0; i < stepVector.size(); i++)
        {
            navStep& s = stepVector[i];
            if(!executeStep(s))
            {
                yCError(NAV_CLIENT_TEST) << "step " << s.label << " failed";
                iNav->stopNavigation();
                return false;
            }
        }

        absLocationTest();

        locationsStored = true;
        iNav->stopNavigation();

        printRegisteredLocations();

        i = 0;
        currentGoal = stepVector.size()-1;
    }

    iNav->getNavigationStatus(status);

    //checking getRelativeLocationOfCurrentTarget()
    static Map2DLocation target_delta;
    iNav->getRelativeLocationOfCurrentTarget(target_delta.x, target_delta.y, target_delta.theta);
    yCDebug(NAV_CLIENT_TEST) << "relative location of current target" << target_delta.x << target_delta.y << target_delta.theta;


    //test suspendNavigation() and resumeNavigation()
    static double currentTime = 0;
    if(currentTime / 100 - floor(currentTime / 100) < 0.001)
    {
        suspResumeTest();
    }
    currentTime++;

    if(status == navigation_status_goal_reached || status == navigation_status_idle)
    {
        /*if(!checkCurrentGoalReached())
        {
            yCError() << "goal " << stepVector[currentGoal].label << " does not match!";

            //yCInfo() << "registered location:" << stepVector[currentGoal].
        }
        else
        {
            yCInfo() << "goal" << stepVector[currentGoal].label << "match!";
        }*/

        currentGoal = i%stepVector.size();
        iNav->gotoTargetByLocationName(stepVector[currentGoal].label);
        Map2DLocation loc, currentLoc;
        iNav->getLocation(stepVector[currentGoal].label, loc);
        iNav->getAbsoluteLocationOfCurrentTarget(currentLoc);
        if(currentLoc != loc)
        {
            yCError(NAV_CLIENT_TEST) << "absolute location of current target given by the navigation server is wrong";
            return false;
        }
        else
        {
            yCInfo(NAV_CLIENT_TEST) << "absolute location of current target successfully tested";
        }
        yCInfo(NAV_CLIENT_TEST) << "goal" << stepVector[(i-1)%stepVector.size()].label << "reached! heading towards goal" << stepVector[currentGoal].label;
        time = Time::now();
        i++;
    }

    if(Time::now() - time > TIMEOUT)
    {
        yCError(NAV_CLIENT_TEST) << "time out while reaching goal " << stepVector[currentGoal].label;
        return false;
    }

    return true;
}

bool checkEqual(double a, double b, double tollerance)
{
    if(fabs(a - b) < tollerance)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool NavTestModule::checkCurrentGoalReached()
{
    Map2DLocation cg;
    navFrame&     f = stepVector[currentGoal].absPos;
    iNav->getCurrentPosition(cg);
    if(checkEqual(cg.x, f.x, linToll) && checkEqual(cg.y, f.y, linToll) && checkEqual(cg.theta, f.t, angToll))
    {
        yCInfo(NAV_CLIENT_TEST) << "tolerance:" << "linear:" << linToll << "angular:" << angToll;
        yCInfo(NAV_CLIENT_TEST) << "difference" << "x:" << cg.x-f.x << "y:" << cg.y-f.y << "t:" << cg.theta-f.t;
        return true;
    }
    else
    {
        yCInfo(NAV_CLIENT_TEST) << "tolerance:" << "linear:" << linToll << "angular:" << angToll;
        yCInfo(NAV_CLIENT_TEST) << "difference" << "x:" << cg.x-f.x << "y:" << cg.y-f.y << "t:" << cg.theta-f.t;
        return false;
    }
}

bool NavTestModule::interruptModule()
{
    if(iNav)
    {
        iNav->stopNavigation();
    }

    return false;
}
