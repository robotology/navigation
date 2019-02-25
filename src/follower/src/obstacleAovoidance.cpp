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

bool bool ObstacleAdvoidance::checkObstaclesInPath()
{

    yarp::sig::Vector scan;
    bool ret = m_iLaser->getLaserMeasurement(m_laser_data);

    if (ret)
    {
        m_las_timeout_counter = 0;
    }
    else
    {
        m_las_timeout_counter++;
        if (m_las_timeout_counter>TIMEOUT_MAX) m_las_timeout_counter = TIMEOUT_MAX;
    }

}


bool ObstacleAdvoidance::checkObstaclesInPath_helper(std::vector<LaserMeasurementData>& laser_data)
{
    //--------------------------------------------------------------------------------------------
    //bool obstacles_class::check_obstacles_in_path(std::vector<LaserMeasurementData>& laser_data)
    //--------------------------------------------------------------------------------------------
    static double last_time_error_message = 0;
    int laser_obstacles  = 0;
    double goal_distance = 1000; //TO BE COMPLETED

    //compute the polygon
    double vertx[4];
    double verty[4];
    double theta              = 0.0;
    //the following 90 degrees rotation is needed to perform the following reference frame rotation.
    //the reference frame of the robot is the one shown on the right.
    //      Y                 X
    //      |       -->       |
    //      O--X           Y--O
    double ctheta             = cos(theta-1.5707);
    double stheta             = sin(theta-1.5707);
    double detection_distance = m_min_detection_distance;

    if (m_enable_dynamic_max_distance)
    {
        //detection_distance is increased from min to max as the velocity of the robot increases
        detection_distance = m_max_detection_distance * m_safety_coeff;
    }

    //an obstacle farther than m_max_detection_distance is always ignored
    if (detection_distance>m_max_detection_distance)
        detection_distance = m_max_detection_distance;

    //an obstacle nearer than m_min_detection_distance is always detected
    if (detection_distance<m_min_detection_distance)
        detection_distance = m_min_detection_distance;

    vertx[0] = (-m_robot_radius) * ctheta + detection_distance * (-stheta);
    verty[0] = (-m_robot_radius) * stheta + detection_distance * ctheta;
    vertx[1] = (+m_robot_radius) * ctheta + detection_distance * (-stheta);
    verty[1] = (+m_robot_radius) * stheta + detection_distance * ctheta;
    vertx[2] = +m_robot_radius  * ctheta;
    verty[2] = +m_robot_radius  * stheta;
    vertx[3] = -m_robot_radius  * ctheta;
    verty[3] = -m_robot_radius  * stheta;

    size_t las_size = laser_data.size();

    if (las_size == 0)
    {
        yError() << "Internal error, invalid laser data struct!";
        return false;
    }

    for (size_t i = 0; i < las_size; i++)
    {
        double d = 0;
        double angle = 0;
        laser_data[i].get_polar(d, angle);

        if (d < m_robot_radius)
        {
            laser_obstacles++;
            if (yarp::os::Time::now() - last_time_error_message > 0.3)
            {
                yError("obstacles on the platform");
                last_time_error_message = yarp::os::Time::now();
            }
            continue;
        }

        double px = 0;
        double py = 0;
        laser_data[i].get_cartesian(px, py);

        if (pnpoly(4,vertx,verty,px,py)>0)
        {
            if (d < goal_distance)
            {
                laser_obstacles++;
                //yError("obstacles on the path");
                continue;
            }
            else
            {
                //yError("obstacles on the path, but goal is near");
                continue;
            }
        }
    }

    //prevent noise to be detected as an obstacle;
    if (laser_obstacles>=2)
    {
        if (yarp::os::Time::now() - m_last_print_time > 1.0)
        {
            yWarning("obstacles detected");
            m_last_print_time = yarp::os::Time::now();
        }
        return true;
    }

    //no obstacles found
    return false;
}
