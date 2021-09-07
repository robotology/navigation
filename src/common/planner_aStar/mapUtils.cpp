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
#include <yarp/dev/Map2DPath.h>
#include <yarp/dev/Map2DLocation.h>
#include <string>
#include <math.h>

#include "mapUtils.h"
#include "aStar.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace map_utilites;

YARP_LOG_COMPONENT(PATHPLAN_MAP, "navigation.devices.robotPathPlanner.map")

bool map_utilites::simplifyPath(MapGrid2D& map, Map2DPath input_path, Map2DPath& output_path)
{
    size_t path_size = input_path.size();
    if (path_size==0) return false;

    output_path.push_back(*input_path.begin());
    
    //make a copy of the path in a vector
    std::vector <XYCell> path;
    for (auto it = input_path.begin(); it!= input_path.end(); it++)
    {
        XYCell tmpcell = map.toXYCell(*it);
        path.push_back(tmpcell);
    }

    for (unsigned int i=0; i<path_size; i++)
    {
        XYCell start_cell = path.at(i);
        XYCell old_stop_cell = start_cell;
        XYCell best_old_stop_cell = start_cell;
        XYCell stop_cell = start_cell;
        XYCell best_stop_cell = start_cell;
        unsigned int j=i+1;
        unsigned int best_j=j;
        for (; j<path_size; j++)
        {
            old_stop_cell = path.at(j-1);
            stop_cell     = path.at(j);
            //yCDebug ("%d %d -> %d %d\n", start_cell.x, start_cell.y, stop_cell.x, stop_cell.y);
            if (checkStraightLine(map, start_cell, stop_cell))
            {
                best_old_stop_cell=old_stop_cell;
                best_stop_cell=stop_cell;
                best_j = j;
            }
        };
        if (best_j==path_size)
        {
            Map2DLocation tmploc = map.toLocation(best_stop_cell);
            output_path.push_back(tmploc);
            return true;
        }
        else
        {
            Map2DLocation tmploc = map.toLocation(best_old_stop_cell);
            output_path.push_back(tmploc);
            i=best_j-1;
        }
    };
    return true;
};

void map_utilites::update_obstacles_map(MapGrid2D& map_to_be_updated, const MapGrid2D& obstacles_map)
{
    //copies obstacles (and only them) from a source map to a destination map
    if (map_to_be_updated.width() != obstacles_map.width() ||
        map_to_be_updated.height() != map_to_be_updated.height())
    {
        yCError(PATHPLAN_MAP) << "update_obstacles_map: the two maps must have the same size!";
        return;
    }
    for (size_t y=0; y<map_to_be_updated.height(); y++)
        for (size_t x=0; x<map_to_be_updated.width(); x++)
        {
            MapGrid2D::map_flags flag_src;
            MapGrid2D::map_flags flag_dst;
            map_to_be_updated.getMapFlag(XYCell (x,y), flag_dst);
            obstacles_map.getMapFlag(XYCell (x,y), flag_src);
            if (flag_dst==MapGrid2D::MAP_CELL_FREE)
            {
                if      (flag_src==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
                { 
                    map_to_be_updated.setMapFlag(XYCell(x, y),MapGrid2D::MAP_CELL_KEEP_OUT);
                }
                else if (flag_src==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE)
                {
                    map_to_be_updated.setMapFlag(XYCell(x, y), MapGrid2D::MAP_CELL_KEEP_OUT);
                }
            }
        }
}

bool map_utilites::checkStraightLine(MapGrid2D& map, XYCell src, XYCell dst)
{
    //here using the fast Bresenham algorithm to check if cells belonging to a straight line (from src to dst)
    //are free or occupied by an obstacle
    int dx = abs(int(dst.x-src.x));
    int dy = abs(int(dst.y-src.y)); 
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

bool map_utilites::findPath(MapGrid2D& map, XYCell start, XYCell goal, Map2DPath& path)
{
    //computes path from start to goal using A* algorithm
    std::deque<XYCell> cell_path;
    bool b = aStar_algorithm::find_astar_path(map, start, goal, cell_path);
    if (b)
    {
        for (auto it = cell_path.begin(); it != cell_path.end(); it++)
        {
            Map2DLocation tmploc = map.toLocation(*it);
            path.push_back(tmploc);
        }
        return true;
    }
    return false;
}

std::vector<yarp::dev::Nav2D::Map2DArea> map_utilites::compute_areas_to_cross(const yarp::dev::Nav2D::Map2DPath& path, const std::vector<yarp::dev::Nav2D::Map2DArea>& Areas)
{
    std::vector<yarp::dev::Nav2D::Map2DLocation> waipoints = path.waypoints;
    std::vector<yarp::dev::Nav2D::Map2DArea> AreasToReturn;
    std::vector<yarp::dev::Nav2D::Map2DArea> AreasTmp = Areas;
    yarp::dev::Nav2D::Map2DLocation location = waipoints[0];
    yarp::dev::Nav2D::Map2DArea area = Areas[0];
    for (int i = 0; i < waipoints.size(); i++)
    {
        location = waipoints[i];
        for (int j = 0; j < AreasTmp.size(); j++)
        {
            area = AreasTmp[j];
            if (area.checkLocationInsideArea(location))
            {
                AreasToReturn.push_back(area);
                AreasTmp.erase(AreasTmp.begin() + j);
                break;
            }
        }
    }
    return AreasToReturn;
}