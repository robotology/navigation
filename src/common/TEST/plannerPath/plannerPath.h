/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef PLANNER_PATH_H
#define PLANNER_PATH_H

#include <deque>
#include <vector>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Map2DLocation.h>

class PlannerPath
{
public:
    class waypoint_t
    {
        public:
        yarp::dev::MapGrid2D::XYCell waypoint_cell;
        yarp::dev::Map2DLocation     waypoint_loc;
    };

private:
    yarp::dev::MapGrid2D::MapInfo     map_info;
    yarp::dev::Map2DLocation          start_point;
    std::deque<waypoint_t>            waypoints;
    yarp::dev::Map2DLocation          final_goal;

public:
    PlannerPath(yarp::dev::MapGrid2D::MapInfo info);
    void pop();
    void push(yarp::dev::Map2DLocation wayp);
    void push(yarp::dev::MapGrid2D::XYCell  wayp);
    void push(waypoint_t  wayp);
    waypoint_t front();

    size_t size();
    void current_waypoint(yarp::dev::MapGrid2D::XYCell& wayp);
    void clear();
    void print();
    bool getPath(std::vector<yarp::dev::Map2DLocation>& path);
};

#endif
