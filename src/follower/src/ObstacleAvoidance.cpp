
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file ObstacleAvoidance.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include "navigation_defines.h"

#include<iostream>

#include "ObstacleAvoidance.h"

YARP_LOG_COMPONENT(FOLLOWER_OBS, "navigation.follower.obstacleAvoidance")

using namespace FollowerTarget;
using namespace FollowerTarget::Obstacle;
using namespace yarp::os;
using namespace yarp::dev;

ObstacleVerifier::ObstacleVerifier(): m_maxDistanceThreshold(MaxDistanceThreshold), m_laser(nullptr),
m_isRunning(false), last_time_error_message(0), m_last_print_time(0), m_debugOn(false)
{;}

bool ObstacleVerifier::configure(yarp::os::ResourceFinder &rf)
{

    bool enabled=false;
    Bottle config_group = rf.findGroup("OBSTACLE_AVOIDANCE");
    if (config_group.isNull())
    {
        yCError(FOLLOWER_OBS) << "Missing OBSTACLE_AVOIDANCE group! the module will not use OBSTACLE_AVOIDANCE fetures!";
    }
    else
    {
        if (config_group.check("enabled"))  {enabled = config_group.find("enabled").asBool(); }
        if (config_group.check("maxDistance"))  {m_maxDistanceThreshold  = config_group.find("maxDistance").asFloat64(); }
        if (config_group.check("robotRadius"))  {m_robotRadius  = config_group.find("robotRadius").asFloat64(); }
        if (config_group.check("robotLaserPortName"))  {m_robotLaserPortName  = config_group.find("robotLaserPortName").asString(); }
    }
    if(!enabled)
        return true;

    if(!initLaserClient(rf))
        return false;

    m_isRunning = true;

    if(m_debugOn)
        yCInfo (FOLLOWER_OBS) << "OBSTACLE_AVOIDANCE has been configured!";
    return true;
}


bool ObstacleVerifier::initLaserClient(yarp::os::ResourceFinder &rf)
{
    // Prepare properties for the Rangefinder2DClient
    yarp::os::Property prop;
    prop.put("device", LIDAR_CLIENT_DEVICE_DEFAULT);
    prop.put("local", "/follower-rangeFinder");
    prop.put("remote", m_robotLaserPortName);

    // Try to open the driver
    bool ok_open = m_driver.open(prop);
    if (!ok_open)
    {
        yCError(FOLLOWER_OBS) << "Unable to open the Rangefinder2DClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_driver.view(m_laser);
    if (!ok_view || m_laser == 0)
    {
        yCError(FOLLOWER_OBS) << "Unable to retrieve the Rangefinder2DClient view.";
        return false;
    }

    if(m_debugOn)
        yCInfo (FOLLOWER_OBS) << "OBSTACLE_AVOIDANCE: Rangefinder2DClient driver has been initialized correctly !";

    //from now I can use m_laser
    return true;

}

bool ObstacleVerifier::isRunning(void)
{
    return m_isRunning;
}


Result ObstacleVerifier::checkObstaclesInPath()
{

    yarp::sig::Vector scan;
    m_laser_data.clear();
    bool ret = m_laser->getLaserMeasurement(m_laser_data);

    Result result;
    if(!ret)
    {
        yCError(FOLLOWER_OBS) << "Error getting laser measurements";
        result.resultIsValid=false;
        return result;
    }

    if(m_laser_data.size()<=0)
    {
        yCError(FOLLOWER_OBS) << "No laser data available. (size=0)";
        result.resultIsValid=false;
        return result;
    }


    result.resultIsValid=true;
    result.result=checkObstaclesInPath_helper(m_laser_data);
    return result;
}


bool ObstacleVerifier::checkObstaclesInPath_helper(std::vector<LaserMeasurementData> & laser_data)
{
    //--------------------------------------------------------------------------------------------
    //bool obstacles_class::check_obstacles_in_path(std::vector<LaserMeasurementData>& laser_data)
    //--------------------------------------------------------------------------------------------


    int laser_obstacles  = 0;
    double goal_distance = m_maxDistanceThreshold;

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
    double detection_distance = m_maxDistanceThreshold;
//Removed: actually not used.
//     if (m_enable_dynamic_max_distance)
//     {
//         //detection_distance is increased from min to max as the velocity of the robot increases
//         detection_distance = m_max_detection_distance * m_safety_coeff;
//     }
//
//     //an obstacle farther than m_max_detection_distance is always ignored
//     if (detection_distance>m_max_detection_distance)
//         detection_distance = m_max_detection_distance;
//
//     //an obstacle nearer thanm_robotRadius m_min_detection_distance is always detected
//     if (detection_distance<m_min_detection_distance)
//         detection_distance = m_min_detection_distance;
//
    vertx[0] = (-m_robotRadius) * ctheta + detection_distance * (-stheta);
    verty[0] = (-m_robotRadius) * stheta + detection_distance * ctheta;
    vertx[1] = (+m_robotRadius) * ctheta + detection_distance * (-stheta);
    verty[1] = (+m_robotRadius) * stheta + detection_distance * ctheta;
    vertx[2] = +m_robotRadius  * ctheta;
    verty[2] = +m_robotRadius  * stheta;
    vertx[3] = -m_robotRadius  * ctheta;
    verty[3] = -m_robotRadius  * stheta;

    size_t las_size = laser_data.size();

    for (size_t i = 0; i < las_size; i++)
    {
        double d = 0;
        double angle = 0;
        laser_data[i].get_polar(d, angle);

        if(std::isinf(d))
        {
            continue;
        }

        if (d < m_robotRadius)
        {
            laser_obstacles++;// I removed Because for the follower its not important to check if there is an obstacle inside the m_robotRadius.
            //We only want to notify the user. VALE: now removed TODO. add info in result
//             if (yarp::os::Time::now() - last_time_error_message > 0.3)
//             {
//                 yCError("obstacles on the platform");
//                 last_time_error_message = yarp::os::Time::now();
//             }
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
                continue;
            }
            else
            {
                if(m_debugOn)
                    yCInfo(FOLLOWER_OBS,"obstacles on the path, but goal is near");
                continue;
            }
        }
    }

    //prevent noise to be detected as an obstacle;
    if (laser_obstacles>=2)
    {
//         if (yarp::os::Time::now() - m_last_print_time > 1.0)
//         {
//             yCWarning("obstacles detected");
//             m_last_print_time = yarp::os::Time::now();
//         }
        return true;
    }

    //no obstacles found
    return false;
}


//checks if a point is inside a polygon
int ObstacleVerifier::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++)
    {
        if (( (verty[i]>testy) != (verty[j]>testy) ) &&
            (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
        {
            c = !c;
        }
    }
    return c;
}

