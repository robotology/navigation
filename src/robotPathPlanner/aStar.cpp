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

#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <string>
#include <list>
#include <math.h>
#include <yarp/dev/MapGrid2D.h>
#include "aStar.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

/////// node_type
node_type::node_type()
{
    empty=true;
    x=0; 
    y=0;
    s_score=0;
    g_score=0;
    f_score=0;
    came_from.x=-1;
    came_from.y=-1;
}

bool operator < (const node_type &a, const node_type &b)
{
    return (a.f_score>b.f_score);
}

/////////// node_map_type
node_map_type::node_map_type(yarp::dev::MapGrid2D& map)
{
    w = map.width();
    h = map.height();
    nodes = new node_type* [w];
    for (int i = 0; i < w; ++i)  nodes[i] = new node_type[h];

    for (int y=0; y<h; y++)
        for (int x=0; x<w; x++)
            {
                if (map.isFree(MapGrid2D::XYCell(x, y)))
                    nodes [x][y].empty = true;
                else
                    nodes [x][y].empty = false;
                nodes [x][y].x = x;
                nodes [x][y].y = y;
                //--- ---
                //s_score is disabled by default.
                //it is can be associated to a particular color code, to generate
                //smooth trajectories,i.e.: keep the robot away from walls, using
                //map skeletonization. The algorithm performances are still to be checked.
                //nodes [x][y].s_score = 230-imgMat.at<cv::Vec3b>(y,x)[1];
                //--- ---
                nodes [x][y].s_score = 0;
            }
}

node_map_type::~node_map_type()
{
    for (int i = 0; i < w; ++i) delete[] nodes[i];
    delete[] nodes;
}

/////////// ordered_set_type
void ordered_set_type::insert (const node_type& t)
{
    set.push_back(t);
    push_heap (set.begin(),set.end()); 
}

node_type ordered_set_type::get_smallest()
{
    node_type t = set.front();
    pop_heap (set.begin(),set.end());
    set.pop_back();
    return t;
}
void ordered_set_type::print()
{
    yDebug("front (smallest)%f \n", set.front().f_score);
    for (unsigned int i=0; i<set.size(); i++)
        yDebug("id%d x%d y%d %f\n", i, set[i].x, set[i].y, set[i].f_score);
    yDebug("back (biggest) %f \n", set.back().f_score);
}

int ordered_set_type::size()
{
    return set.size();
}

bool ordered_set_type::find(node_type t)
{
    for (unsigned int i=0; i<set.size(); i++)
    {
        if (set[i].x == t.x &&
            set[i].y == t.y)
            return true;
    }
    return false;
}

/////////// unordered_set_type
void unordered_set_type::insert (const node_type& t)
{
    set.push_back(t);
}
    
bool unordered_set_type::find(node_type t)
{
    for (unsigned int i=0; i<set.size(); i++)
    {
        if (set[i].x == t.x &&
            set[i].y == t.y)
            return true;
    }
    return false;
}

/////////// various
bool find_astar_path(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell start, MapGrid2D::XYCell goal, std::queue<MapGrid2D::XYCell>& path)
{
    std::vector<MapGrid2D::XYCell> inverse_path;
    node_map_type node_map(map);
    int sx=start.x;
    int sy=start.y;
    int gx=goal.x;
    int gy=goal.y;

    unordered_set_type closed_set;
    ordered_set_type   open_set;  

    open_set.insert(node_map.nodes[sx][sy]);
    
    node_map.nodes[sx][sy].g_score = 0;
    node_map.nodes[sx][sy].f_score = node_map.nodes[sx][sy].g_score + heuristic_cost_estimate(node_map.nodes[sx][sy], node_map.nodes[gx][gy]);

    int iterations=0;
    while (open_set.size()>0)
    {
        iterations++;
        //yDebug ("%d\n", iterations++);
        //open_set.print();
        node_type curr=open_set.get_smallest();
        
        if (curr.x==goal.x &&
            curr.y==goal.y) 
            {
                MapGrid2D::XYCell c;
                c.x=goal.x;
                c.y=goal.y;
                while (!(c.x==start.x && c.y==start.y))
                {
                    inverse_path.push_back(c);
                    int old_cx = c.x;
                    int old_cy = c.y;
                    c.x = node_map.nodes[old_cx][old_cy].came_from.x;
                    c.y = node_map.nodes[old_cx][old_cy].came_from.y;
                }

                //reverse the path
                for (int i=inverse_path.size()-1; i>=0; i--)
                {
                    path.push(inverse_path[i]);
                }
                return true;
            }

        closed_set.insert(curr);

        list<node_type> neighbors;
        //yDebug ("%d %d \n", curr.x, curr.y);
        if (node_map.nodes[curr.x][curr.y + 1].empty)   neighbors.push_back(node_map.nodes[curr.x][curr.y + 1]);
        if (node_map.nodes[curr.x][curr.y - 1].empty)   neighbors.push_back(node_map.nodes[curr.x][curr.y - 1]);
        if (node_map.nodes[curr.x + 1][curr.y].empty)   neighbors.push_back(node_map.nodes[curr.x + 1][curr.y]);
        if (node_map.nodes[curr.x - 1][curr.y].empty)   neighbors.push_back(node_map.nodes[curr.x - 1][curr.y]);
        if (node_map.nodes[curr.x + 1][curr.y + 1].empty) neighbors.push_back(node_map.nodes[curr.x + 1][curr.y + 1]);
        if (node_map.nodes[curr.x + 1][curr.y - 1].empty) neighbors.push_back(node_map.nodes[curr.x + 1][curr.y - 1]);
        if (node_map.nodes[curr.x - 1][curr.y + 1].empty) neighbors.push_back(node_map.nodes[curr.x - 1][curr.y + 1]);
        if (node_map.nodes[curr.x - 1][curr.y - 1].empty) neighbors.push_back(node_map.nodes[curr.x - 1][curr.y - 1]);
        /*int cl = curr.x-1>0?curr.x-1:0;
        int cu = curr.y-1>0?curr.y-1:0;
        int cr = curr.x+1<node_map.w?curr.x+1:node_map.w-1;
        int cd = curr.y+1<node_map.h?curr.y+1:node_map.h-1;

        neighbors.push_back(node_map.nodes[curr.x][cd]);
        neighbors.push_back(node_map.nodes[curr.x][cu]);
        neighbors.push_back(node_map.nodes[cr][curr.y]);
        neighbors.push_back(node_map.nodes[cl][curr.y]);
        neighbors.push_back(node_map.nodes[cr][cd]);
        neighbors.push_back(node_map.nodes[cr][cu]);
        neighbors.push_back(node_map.nodes[cl][cd]);
        neighbors.push_back(node_map.nodes[cl][cd]);*/
        //yDebug ("%d %d \n", curr.x, curr.y);

        while(neighbors.size()>0)
        {
            node_type neighbor = neighbors.front();

            if (closed_set.find(neighbor) || !neighbor.empty)
            {
                neighbors.pop_front();
                continue;
            }
            
            int nx = neighbor.x;
            int ny = neighbor.y;
            double tentative_g_score=0;
            if (neighbor.empty)
            {
                // add the distance between curr and neigh
                if ( (nx==curr.x+1 && ny==curr.y   ) ||
                     (nx==curr.x-1 && ny==curr.y   ) ||
                     (nx==curr.x   && ny==curr.y+1 ) ||
                     (nx==curr.x   && ny==curr.y-1 ) ) tentative_g_score = curr.g_score + 10 + curr.s_score;
                else 
                    tentative_g_score = curr.g_score + 14 + curr.s_score;     
            }
            else
                tentative_g_score = curr.g_score + 1e10 + curr.s_score;
            
            bool b = open_set.find(neighbor);
            if (!b || tentative_g_score < node_map.nodes[nx][ny].g_score)
            {
                node_map.nodes[nx][ny].came_from.x = curr.x;
                node_map.nodes[nx][ny].came_from.y = curr.y;
                node_map.nodes[nx][ny].g_score = tentative_g_score;
                node_map.nodes[nx][ny].f_score = node_map.nodes[nx][ny].g_score + heuristic_cost_estimate(node_map.nodes[neighbor.x][neighbor.y], node_map.nodes[gx][gy]);
                if (!b)
                {
                    open_set.insert(node_map.nodes[nx][ny]);
                }
            }

            neighbors.pop_front();
        }
    };

    //no path found
    return false;
}

double heuristic_cost_estimate (node_type start, node_type goal)
{
    double dist = sqrt ( double((start.x-goal.x)*(start.x-goal.x) +
                                (start.y-goal.y)*(start.y-goal.y)))*10;
    //double dist = 0;
    return dist;
}

