#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include "navigation_defines.h"

typedef struct navigationTestFrame
{
    navigationTestFrame(){x = 0; y = 0; t = 0;}
    navigationTestFrame(double ix, double iy, double it)
    {
        x = ix;
        y = iy;
        t = it;
    }

    double x;
    double y;
    double t;
}navFrame;

typedef struct navigationTestStep
{
    navigationTestStep()
    {
        label = "unamed";
    }

    std::vector<navFrame>    frames;
    std::string              label;
    navFrame                 absPos;

}navStep;

//This module is used to test internally interface functionalities
class NavTestModule : public yarp::os::RFModule
{
private:
    double                     period;

    //device drivers and interfaces
    yarp::dev::PolyDriver      ddNavClient;
    yarp::dev::PolyDriver      ddLocServer;
    yarp::dev::Nav2D::INavigation2D*  iNav;
    std::string m_nameof_remote_localization_port =LOCALIZATION_REMOTE_PORT_DEFAULT;
    std::string m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;

    //goal/location variables
    std::vector<navStep>       stepVector;
    unsigned int               currentGoal;
    double                     linToll;
    double                     angToll;
    bool                       locationsStored;

    //tests
    bool                       checkCurrentGoalReached();
    bool                       executeStep(navStep s);
    inline void                absLocationTest();
    inline void                suspResumeTest();

    //debug methods
    void                       printRegisteredLocations();

public:
    //constructor/destructor
    NavTestModule();
    ~NavTestModule();

protected:
    //methods inherited from yarp::os::RFModule
    virtual bool    configure(yarp::os::ResourceFinder& rf );
    virtual bool    close();
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual bool    interruptModule();
};
