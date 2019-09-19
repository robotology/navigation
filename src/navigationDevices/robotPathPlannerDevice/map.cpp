/* 
* Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
* All rights reserved.
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it

 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/Map2DLocation.h>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h> 
#include <plannerPath.h>

#include "map.h"
#include "aStar.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace map_utilites;

bool map_utilites::simplifyPath(yarp::dev::MapGrid2D& map, PlannerPath input_path, PlannerPath& output_path)
{
    unsigned int path_size = input_path.size();
    if (path_size==0) return false;

    output_path.push(input_path.front());
    
    //make a copy of the path in a vector
    std::vector <MapGrid2D::XYCell> path;
    for (unsigned int i=0; i<path_size; i++)
    {
        PlannerPath::waypoint_t tmp = input_path.front();
        input_path.pop();
        path.push_back(tmp.waypoint_cell);
    }

    for (unsigned int i=0; i<path_size; i++)
    {
        MapGrid2D::XYCell start_cell = path.at(i);
        MapGrid2D::XYCell old_stop_cell = start_cell;
        MapGrid2D::XYCell best_old_stop_cell = start_cell;
        MapGrid2D::XYCell stop_cell = start_cell;
        MapGrid2D::XYCell best_stop_cell = start_cell;
        unsigned int j=i+1;
        unsigned int best_j=j;
        for (; j<path_size; j++)
        {
            old_stop_cell = path.at(j-1);
            stop_cell     = path.at(j);
            //yDebug ("%d %d -> %d %d\n", start_cell.x, start_cell.y, stop_cell.x, stop_cell.y);
            if (checkStraightLine(map, start_cell, stop_cell))
            {
                best_old_stop_cell=old_stop_cell;
                best_stop_cell=stop_cell;
                best_j = j;
            }
        };
        if (best_j==path_size)
        {
            output_path.push(best_stop_cell);
            return true;
        }
        else
        {
            output_path.push(best_old_stop_cell);
            i=best_j-1;
        }
    };
    return true;
};

void map_utilites::update_obstacles_map(yarp::dev::MapGrid2D& map_to_be_updated, const yarp::dev::MapGrid2D& obstacles_map)
{
    //copies obstacles (and only them) from a source map to a destination map
    if (map_to_be_updated.width() != obstacles_map.width() ||
        map_to_be_updated.height() != map_to_be_updated.height())
    {
        yError() << "update_obstacles_map: the two maps must have the same size!";
        return;
    }
    for (size_t y=0; y<map_to_be_updated.height(); y++)
        for (size_t x=0; x<map_to_be_updated.width(); x++)
        {
            MapGrid2D::map_flags flag_src;
            MapGrid2D::map_flags flag_dst;
            map_to_be_updated.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag_dst);
            obstacles_map.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag_src);
            if (flag_dst==MapGrid2D::MAP_CELL_FREE)
            {
                if      (flag_src==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE; }
                else if (flag_src==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE; }
            }
        }
}

bool map_utilites::checkStraightLine(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell src, MapGrid2D::XYCell dst)
{
    //here using the fast Bresenham algorithm to check if cells belonging to a straight line (from src to dst)
    //are free or occupied by an obstacle
    int dx = abs(dst.x-src.x);
    int dy = abs(dst.y-src.y); 
    int err = dx-dy;
    
    int sx;
    int sy;
    if (src.x < dst.x) sx = 1; else sx = -1;
    if (src.y < dst.y) sy = 1; else sy = -1;
    
    while(1)
    {
        if (map.isFree(src) == false) return false;
        if (src.x==dst.x && src.y==dst.y) break;
        int e2 = err*2;
        if (e2 > -dy)
        {
            err = err-dy;
            src.x += sx;
        }
        if (e2 < dx)
        {
            err = err+dx;
            src.y += sy;
        }
    }
    return true;
}

bool map_utilites::findPath(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell start, MapGrid2D::XYCell goal, PlannerPath& path)
{
    //computes path from start to goal using A* algorithm
    std::deque<MapGrid2D::XYCell> cell_path;
    bool b = aStar_algorithm::find_astar_path(map, start, goal, cell_path);
    if (b)
    {
        for (auto it = cell_path.begin(); it != cell_path.end(); it++)
        {
            path.push(*it);
        }
        return true;
    }
    return false;
}
