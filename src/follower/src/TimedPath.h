/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file TimedPath.h
 * @authors: Marco Randazzo <marco.randazzo@iit.it>, Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef TIMEDPATH_H
#define TIMEDPATH_H

#include <list>

#include <yarp/dev/Map2DLocation.h>

namespace FollowerTarget
{

    class TimedPath
    {
        struct TimedWaypoint_t
        {
            double time;
            yarp::dev::Map2DLocation loc;
        };

        std::list<TimedWaypoint_t> path;

    public:
        double sampling_time;
        double max_queue_lenght;

        TimedPath();

        yarp::dev::Map2DLocation getPrediction(void);

        void update(yarp::dev::Map2DLocation cur_loc);
    };

}

#endif
