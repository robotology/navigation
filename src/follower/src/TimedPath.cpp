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

#include "TimedPath.h"

#include <yarp/os/Time.h>

using namespace yarp::dev;
using namespace FollowerTarget;



TimedPath::TimedPath()
{
    sampling_time = 1; //sec
    max_queue_lenght = 5; //elements
}


Map2DLocation TimedPath::getPrediction(void)
{
    if (path.size() == 0)
    {
        Map2DLocation tmp;
        return tmp;
    }
    else if (path.size() == 1)
    {
        return path.front().loc;
    }

    //path size is at least two
    auto it1 = path.begin();
    auto it2 = it1++;
    double diff_x = it1->loc.x - it2->loc.x;
    double diff_y = it1->loc.y - it2->loc.y;

    Map2DLocation est = (it1->loc);
    est.x = est.x + diff_x;
    est.y = est.y + diff_y;
    return est;
}

void TimedPath::update(yarp::dev::Map2DLocation cur_loc)
{
    double cur_time = yarp::os::Time::now();

    if (path.size() == 0)
    {
        TimedWaypoint_t new_waypoint;
        new_waypoint.time = cur_time;
        new_waypoint.loc = cur_loc;
        path.push_back(new_waypoint);
        return;
    }

    if (cur_time - path.front().time < sampling_time)
    {
        return;
    }

    TimedWaypoint_t new_waypoint;
    new_waypoint.time = cur_time;
    new_waypoint.loc = cur_loc;
    path.push_back(new_waypoint); //add new element

    if (path.size() > max_queue_lenght)
    {
        path.pop_front(); //remove an element
    }
}
