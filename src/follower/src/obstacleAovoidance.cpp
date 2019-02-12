#include "ObstacleAdvoidance.h"

ObstacleAdvoidance::ObstacleAdvoidance(): m_maxDistanceThreshold(m_defaultMaxDistanceThreshold),
m_laser(nullptr), m_isRunning(false)
{;}

bool ObstacleAdvoidance::configure(yarp::os::ResourceFinder &rf)
{

    bool enabled=false;
    Bottle config_group = rf.findGroup("OBSTACLE_AVOIDANCE");
    if (config_group.isNull())
    {
        yError() << "Missing OBSTACLE_AVOIDANCE group! the module will not use OBSTACLE_AVOIDANCE!";
    }
    else
    {
        if (config_group.check("enabled"))  {enabled = config_group.find("enabled").asBool(); }
        if (config_group.check("maxDistance"))  {m_maxDistanceThreshold  = config_group.find("maxDistance").asDouble(); }

    }
    if(!enabled)
        return true;

    if(!initLaserClient(rf))
        return false;

    m_isRunning = true;
    return true;
}


bool ObstacleAdvoidance::initLaserClient(yarp::os::ResourceFinder &rf)
{
    // Prepare properties for the Rangefinder2DClient
    yarp::os::Property prop;
    prop.put("device", "Rangefinder2DClient");
    prop.put("local", "/follower-rangeFinder");
    prop.put("remote", "/rangeFinderServer");

    // Try to open the driver
    bool ok_open = m_driver.open(prop);
    if (!ok_open)
    {
        yError() << "Unable to open the Rangefinder2DClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_driver.view(m_laser);
    if (!ok_view || m_laser == 0)
    {
        yError() << "Unable to retrieve the Rangefinder2DClient view.";
        return false;
    }

    //from now I can use m_laser
    return true;

}

bool ObstacleAdvoidance::isRunning(void)
{return m_isRunning;}
double ObstacleAdvoidance::calculateWeightLaserVectors(void)
{
    return 0.0;
}
