/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef A_STAR_H
#define A_STAR_H

#include <yarp/dev/MapGrid2D.h>

#include <vector>
#include <queue>

//! namespace containing a complete implementation of the classic A* algorithm
namespace aStar_algorithm
{
    /**
    * This method computes (if exists) the path required to go from a start cell to a goal cell
    * @param map the gridmap containing the obstacles
    * @param start the start cell(x,y)
    * @param goal the arrival cell(x,y)
    * @param path the computed sequence of cells required to go from  start cell to goal cell
    * @return true if the path exists, false if no valid path has been found
    */
    bool find_astar_path(yarp::dev::Nav2D::MapGrid2D& map, yarp::dev::Nav2D::XYCell start, yarp::dev::Nav2D::XYCell goal, std::deque<yarp::dev::Nav2D::XYCell>& path);
};

#endif
