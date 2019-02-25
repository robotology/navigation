#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IRangefinder2D.h>

class ObstacleAdvoidance
{
public:
    ObstacleAdvoidance();
    //return false in case of error, else true
    bool configure(yarp::os::ResourceFinder &rf);
    //return true if it has been enabled
    bool isRunning();
    bool checkObstaclesInPath(void);
private:
    double m_maxDistanceThreshold;
    double const m_defaultMaxDistanceThreshold = 2.5; //meters

    yarp::dev::IRangefinder2D * m_laser;
    yarp::dev::PolyDriver      m_driver;
    bool m_isRunning;

    bool initLaserClient(yarp::os::ResourceFinder &rf);
    double calculateWeightLaserVectors(void);
    bool checkObstaclesInPath_helper(std::vector<LaserMeasurementData>& laser_data);
};
