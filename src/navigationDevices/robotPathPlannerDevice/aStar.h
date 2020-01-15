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
