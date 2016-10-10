#include "navigationtestmodule.h"
#include <cmath>
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;
#define PERIOD  0
#define TIMEOUT 60
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
    Time::now();

    pLocationServer_cfg.put("device", "locationsServer");
    pLocationServer_cfg.put("local", "/locationServer");
    pLocationServer_cfg.put("ROS_enabled", "");
    bool ok_location = ddLocServer.open(pLocationServer_cfg);
    if(ok_location){yInfo() << "ddLocationServer open reported successful";};

    navTestCfg.put("device",         "navigation2DClient");
    navTestCfg.put("local",          "/navigationTest");
    navTestCfg.put("remote",         "/robotGoto");
    navTestCfg.put("locationRemote", "/locationServer");

    iNav            = 0;
    okClient        = ddNavClient.open(navTestCfg);
    okView          = ddNavClient.view(iNav);

    if(okClient)
    {
        yInfo("ddNavClinet device open reported successful");
        if(okView)
        {
            yInfo("INavigation2D interface open reported successful");
        }
        else
        {
            yError("Error opening INavigation2D interface");
            return false;
        }
    }
    else
    {
        yError("Error opening ddNavClinet device");
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
        yError("navigation interface pointer is not valid anymore");
        return false;
    }

    for(i = 0; i < s.frames.size(); i++)
    {
        navFrame& f = s.frames[i];
        yInfo() << "reaching location" << s.label << ". heading towards step n." << i << "located in (relative) x:" << f.x << "y:" << f.y << "th:" << f.t << "for registration purpose";
        iNav->gotoTargetByRelativeLocation(f.x,f.y,f.t);
        iNav->getNavigationStatus(status);
        time = Time::now();
        while (status != navigation_status_goal_reached)
        {
            yarp::os::Time::delay(0.1);
            if(!iNav->getNavigationStatus(status))
            {
                yError("unable to get navigation status while heading towards frame %lu of step %s", i, s.label.c_str());
                return false;
            }
            if(Time::now() - time >= TIMEOUT)
            {
                yError("time out while heading towards frame %lu of step %s ", i, s.label.c_str());
                return false;
            }
        }
    }

    if(!iNav->storeCurrentPosition(s.label))
    {
        yError() << "error storing location" << s.label;
    }
    return true;
}

void NavTestModule::printRegisteredLocations()
{
    std::vector<yarp::os::ConstString>    locations;
    std::vector<yarp::os::ConstString>    emptyLocationList;
    std::vector<yarp::dev::Map2DLocation> coords;
    if(!iNav->getLocationsList(locations))
    {
        yError() << "error retrieving location list";
    }

    if(locations.size() == 0)
    {
        yInfo() << "no locations registered";
    }

    for(size_t i = 0; i < locations.size(); i++)
    {
        yarp::dev::Map2DLocation l;
        if(!iNav->getLocation(locations[i], l))
        {
            yError() << "error retrieving location" << locations[i];
        }
        else
        {
            coords.push_back(l);
            yInfo() << locations[i] << " is located in x: " << l.x << " y: " << l.y << " and theta: " << l.theta;
        }
    }
    yInfo() << "Test: deleting 1st location";
    if(iNav->deleteLocation(locations[0]))
    {
        yInfo() << "Succesful!!";
    }
    else
    {
        yError() << "miserably failed..";
    }

    yInfo() << "Test: deleting every stored location";
    if(iNav->clearAllLocations())
    {
        yInfo() << "Succesful!!";
    }
    else
    {
        yError() << "miserably failed..";
    }

    iNav->getLocationsList(emptyLocationList);

    if(emptyLocationList.size())
    {
        yError() << "error.. location server actually contains" << emptyLocationList.size() << "locations";
    }

    yInfo() << "Test: re-storing location removed";
    for(size_t i = 0; i < locations.size(); i++)
    {
        if(!iNav->storeLocation(locations[i], coords[i]))
        {
            yError() << "error re-storing location" << locations[i];
        }
        else
        {
            yInfo() << "Location" << locations[i] << "succesfully re-stored";
        }
    }

    for(size_t i = 0; i < locations.size(); i++)
    {
        yarp::dev::Map2DLocation l;
        iNav->getLocation(locations[i], l);
        yDebug() << locations[i] << " is located in x: " << l.x << " y: " << l.y << " and theta: " << l.theta;
    }

}

void NavTestModule::absLocationTest()
{
    yInfo() << "Test: go to by absolute location:";
    double time = Time::now();
    if(!iNav->gotoTargetByAbsoluteLocation( yarp::dev::Map2DLocation("map",0,0,0) ) )
    {
        yError() << "failed!";
    }
    NavigationStatusEnum status;
    do
    {
        iNav->getNavigationStatus(status);
        yarp::os::Time::delay(0.1);
        if(Time::now() - time >= TIMEOUT)
        {
            yError("time out!!");
        }
    }
    while(status != navigation_status_goal_reached );
}

void NavTestModule::suspResumeTest()
{
    yInfo() << "Test: suspending navigation";
    if(iNav->suspendNavigation())
    {
        yInfo() << "Succesful!!";
    }
    else
    {
        yError() << "miserably failed..";
    }

    yInfo() << "5 second delay..";
    for(size_t i = 0; i < 5; i ++)
    {
        yDebug() << i;
        yarp::os::Time::delay(1);
    }
    yInfo() << "Test: resuming navigation";
    if(iNav->resumeNavigation())
    {
        yInfo() << "Succesful!!";
    }
    else
    {
        yError() << "miserably failed..";
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
        yarp::dev::Map2DLocation initialPos;
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
                yError() << "step " << s.label << " failed";
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
    yDebug() << "relative location of current target" << target_delta.x << target_delta.y << target_delta.theta;


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
            yError() << "goal " << stepVector[currentGoal].label << " does not match!";

            //yInfo() << "registered location:" << stepVector[currentGoal].
        }
        else
        {
            yInfo() << "goal" << stepVector[currentGoal].label << "match!";
        }*/

        currentGoal = i%stepVector.size();
        iNav->gotoTargetByLocationName(stepVector[currentGoal].label);
        Map2DLocation loc, currentLoc;
        iNav->getLocation(stepVector[currentGoal].label, loc);
        iNav->getAbsoluteLocationOfCurrentTarget(currentLoc);
        if(currentLoc != loc)
        {
            yError() << "absolute location of current target given by the navigation server is wrong";
            return false;
        }
        else
        {
            yInfo() << "absolute location of current target succesfully tested";
        }
        yInfo() << "goal" << stepVector[(i-1)%stepVector.size()].label << "reached! heading towards goal" << stepVector[currentGoal].label;
        time = Time::now();
        i++;
    }

    if(Time::now() - time > TIMEOUT)
    {
        yError() << "time out while reaching goal " << stepVector[currentGoal].label;
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
        yInfo() << "tollerance:" << "linear:" << linToll << "angular:" << angToll;
        yInfo() << "difference" << "x:" << cg.x-f.x << "y:" << cg.y-f.y << "t:" << cg.theta-f.t;
        return true;
    }
    else
    {
        yInfo() << "tollerance:" << "linear:" << linToll << "angular:" << angToll;
        yInfo() << "difference" << "x:" << cg.x-f.x << "y:" << cg.y-f.y << "t:" << cg.theta-f.t;
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
