#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include "navigation_defines.h"

//#include <yarp/dev/IFrameTransform.h>
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace std;

YARP_LOG_COMPONENT(NAV_CLIENT_SNIPPET, "navigation.navigation2DClientSnippet")

int main(int argc, char *argv[])
{
    Network yarp;

    string m_nameof_remote_navigation_port = "/robotPathPlanner";
    string m_nameof_remote_map_port = MAP_REMOTE_PORT_DEFAULT;
    string m_nameof_remote_localization_port = LOCALIZATION_REMOTE_PORT_DEFAULT;

    //opens a navigation2DClient device and gets a INavigation2D interface.
    Property navTestCfg;
    navTestCfg.put("device",               NAVIGATION_CLIENT_DEVICE_DEFAULT);
    navTestCfg.put("local",                "/navigationTest_navClient");
    navTestCfg.put("navigation_server",    m_nameof_remote_navigation_port);
    navTestCfg.put("map_locations_server", m_nameof_remote_map_port);
    navTestCfg.put("localization_server",  m_nameof_remote_localization_port);
    yarp::dev::PolyDriver     ddNavClient;
    yarp::dev::Nav2D::INavigation2D* iNav                 = 0;
    bool okClient             = ddNavClient.open(navTestCfg);
    bool okView               = ddNavClient.view(iNav);
    if(!okClient || !okView)
    {
        yCError(NAV_CLIENT_SNIPPET,"Error opening INavigation2D interface");
        return false;
    }

    //opens a map2DClient device and gets a iMap2D interface.
    Property map_options;
    map_options.put("device", MAP_CLIENT_DEVICE_DEFAULT);
    map_options.put("local", "/navigationTest_mapClient");
    map_options.put("remote", m_nameof_remote_map_port);
    PolyDriver      ddMapClient;
    IMap2D* iMap    = 0;
    okClient             = ddMapClient.open(map_options);
    okView               = ddMapClient.view(iMap);
    if(!okClient || !okView)
    {
        yCError(NAV_CLIENT_SNIPPET, "Error opening IMap2D interface");
        return false;
    }

    const double TIMEOUT = 30.0; //seconds
    
    //Stops the navigation server, if it is executing some previously launched task.
    iNav->stopNavigation();

    //defines a new location called location_1 and saves it to map server
    Map2DLocation location_1;
    location_1.map_id="testMap";
    location_1.x=2.0;
    location_1.x=1.0;
    location_1.theta=45.0;
    iMap->storeLocation("location_1",location_1);

    //starts the navigation task
    iNav->gotoTargetByLocationName("location_1");
            
    double init_time = Time::now();
    NavigationStatusEnum status;
    do 
    {
        //gets the current position of the robot and displays it
        Map2DLocation current_position;
        iNav->getCurrentPosition(current_position);
        yCInfo(NAV_CLIENT_SNIPPET) << "Current robot position is: " << current_position.toString();

        //gets the navigation status
        iNav->getNavigationStatus(status);
        if(!iNav->getNavigationStatus(status))
        {
            yCError(NAV_CLIENT_SNIPPET) << "Unable to get navigation status";
            break;
        }

        //continue navigation until the goal is reached (or the timeout is expired)
        if (status == navigation_status_goal_reached)
        {
            yCInfo(NAV_CLIENT_SNIPPET) << "Goal reached!";
            break;
        }
        else if(Time::now() - init_time >= TIMEOUT)
        {
            yCError(NAV_CLIENT_SNIPPET) << "Timeout while heading towards current waypoint";
            break;
        }

        //sleep
        yarp::os::Time::delay(0.1);
    }
    while (1);

    //closes the opened device drivers
    ddNavClient.close();
    ddMapClient.close();

    return 0;
}
