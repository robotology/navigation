#include "navigationtestmodule.h"
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;
#define I2S(x) static_cast<ostringstream*>( &(ostringstream() << x) )->str()
NavTestModule::NavTestModule()
{
    period          = PERIOD;
    locationsStored = false;
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
    Property        navTestCfg;

    navTestCfg.put("device", "Navigation2DClient");
    navTestCfg.put("local", "/navigationTest");
    navTestCfg.put("remote", "/navigationServer");

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
                yInfo() << "cippa";
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

        step.frames.push_back(navFrame(0, 1, 90));
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
        i++;
    }


    return true;
}

bool NavTestModule::checkCurrentGoalReached()
{
    Map2DLocation cg;
    navFrame&     f = stepVector[currentGoal].frames.back();
    iNav->getCurrentPosition(cg);
    if(cg.x == f.x && cg.y == f.y && cg.theta == f.t)
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
    return false;
}
