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

class node_type
{
    public:
    bool empty;
    int x;
    int y;
    double g_score;
    double f_score;
    double s_score;
    yarp::dev::MapGrid2D::XYCell came_from;
    node_type();
    friend bool operator<  (const node_type &a, const node_type &b);
};

bool operator < (const node_type &a, const node_type &b);

class node_map_type
{
    public:
    node_map_type();
    node_map_type(yarp::dev::MapGrid2D& map);
    ~node_map_type();

    public:
    int w;
    int h;
    node_type** nodes;
};

double heuristic_cost_estimate (node_type start, node_type goal);

class ordered_set_type
{
    std::vector<node_type> set;

    public:
    void insert (const node_type& t);
    node_type get_smallest();
    void print();
    int size();
    bool find(node_type t);
};

class unordered_set_type
{
    std::vector<node_type> set;

    public:
    void insert (const node_type& t);
    bool find(node_type t);
};

bool find_astar_path(yarp::dev::MapGrid2D& map, yarp::dev::MapGrid2D::XYCell start, yarp::dev::MapGrid2D::XYCell goal, std::queue<yarp::dev::MapGrid2D::XYCell>& path);

#endif
