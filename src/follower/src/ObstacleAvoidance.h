
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file ObstacleAvoidance.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef OBSTACLEAVOIDANCE_H
#define OBSTACLEAVOIDANCE_H

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/LaserMeasurementData.h>

#include <vector>

namespace FollowerTarget
{
    namespace Obstacle
    {
        class Result
        {
        public:
            bool result;
            bool resultIsValid;
            Result(){resultIsValid=false; result=false;}
            //TODO: make this movable
        };

        double const MaxDistanceThreshold = 0.6; //meters NOTE: the target can be at most 1 meter.
        double const RobotRadius =0.3; //meters

        class ObstacleVerifier
        {
        public:
            ObstacleVerifier();
            //return false in case of error, else true
            bool configure(yarp::os::ResourceFinder &rf);
            //return true if it has been enabled
            bool isRunning();
            Result checkObstaclesInPath();
        private:
            double m_maxDistanceThreshold;;
            double m_robotRadius;

            yarp::dev::IRangefinder2D * m_laser;
            yarp::dev::PolyDriver      m_driver;
            bool m_isRunning;
            std::vector<yarp::dev::LaserMeasurementData>  m_laser_data;

            double last_time_error_message;
            double m_last_print_time;

            bool initLaserClient(yarp::os::ResourceFinder &rf);
            bool checkObstaclesInPath_helper(std::vector<yarp::dev::LaserMeasurementData>& laser_data);
            int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
        };
    }
}
#endif

