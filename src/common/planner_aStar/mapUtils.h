/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MAP_H
#define MAP_H

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/MapGrid2D.h>
#include <string>
#include <queue>

using namespace std;
using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265
#endif

//! Helper functions which operates on a map grid, computing a path, drawing an image etc.
namespace map_utilites
{
    //return true if the straight line that connects src with dst does not contain any obstacles
    bool checkStraightLine(yarp::dev::Nav2D::MapGrid2D& map, yarp::dev::Nav2D::XYCell src, yarp::dev::Nav2D::XYCell dst);

    //simplify the path
    bool simplifyPath(yarp::dev::Nav2D::MapGrid2D& map, yarp::dev::Nav2D::Map2DPath input_path, yarp::dev::Nav2D::Map2DPath& output_path);

    //compute a path, given a start cell, a goal cell and a map grid.
    bool findPath(yarp::dev::Nav2D::MapGrid2D& map, yarp::dev::Nav2D::XYCell start, yarp::dev::Nav2D::XYCell goal, yarp::dev::Nav2D::Map2DPath& path);

    // register new obstacles into a map
    void update_obstacles_map(yarp::dev::Nav2D::MapGrid2D& map_to_be_updated, const yarp::dev::Nav2D::MapGrid2D& obstacles_map);

    // computes a list of areas crossed by given path
    std::vector<yarp::dev::Nav2D::Map2DArea> compute_areas_to_cross(const yarp::dev::Nav2D::Map2DPath& path, const std::vector<yarp::dev::Nav2D::Map2DArea>& Areas);
};

#endif
