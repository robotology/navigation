#include "navigationtestmodule.h"
#include <cmath>
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;
#define PERIOD  0
#define TIMEOUT 600
NavTestModule::NavTestModule()
{
    period          = PERIOD;
    locationsStored = false;
    linToll = 0.1;
    angToll = 0.1;
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
        iNav->gotoTargetByRelativeLocation(f.x,f.y,f.t);
        time = Time::now();
        while (status != navigation_status_goal_reached)
        {
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

    iNav->storeCurrentPosition(s.label);
    return true;
}

bool NavTestModule::updateModule()
{
    static size_t        i;
    NavigationStatusEnum status;
    double               time;

    //storing location
    if(!locationsStored)
    {
        navStep step;

        step.frames.push_back(navFrame(0,  -0.5, 0));
        step.frames.push_back(navFrame(0,   0,   90));
        step.frames.push_back(navFrame(0.5, 0,   0));
        step.absPos = navFrame(0.5, -0.5, 0);
        step.label  = "NE";
        stepVector.push_back(step);

        step.frames.push_back(navFrame(0, 1, 0));
        step.absPos = navFrame(0.5, 0.5, 0);
        step.label  = "NW";
        stepVector.push_back(step);
        step.absPos = navFrame(-0.5, 0.5, 0);
        step.label = "SW";
        stepVector.push_back(step);
        step.absPos = navFrame(-0.5, -0.5, 0);
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

        locationsStored = true;
        i = 0;
        currentGoal = stepVector.size()-1;
    }

    //navigating trough location
    iNav->getNavigationStatus(status);
    if(status == navigation_status_goal_reached)
    {
        if(!checkCurrentGoalReached())
        {
            yError() << "goal " << stepVector[currentGoal].label << " does not match!";
        }

        currentGoal = i%stepVector.size();
        iNav->gotoTargetByLocationName(stepVector[currentGoal].label);
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
        return true;
    }
    else
    {
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
