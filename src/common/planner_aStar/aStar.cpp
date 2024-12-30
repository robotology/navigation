/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
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
using namespace yarp::dev::Nav2D;
using namespace aStar_algorithm;

YARP_LOG_COMPONENT(PATHPLAN_ASTAR, "navigation.devices.robotPathPlanner.aStar")

namespace aStar_algorithm
{
    //forward declaration for heuristic_cost_estimate function() 
    class node_type;

    /**
    * This method returns the cost to transverse a map from start node to goal node.
    * @param start the start node
    * @param goal the arrival node
    * @return the cost (euclidean distance * 10)
    */
    double heuristic_cost_estimate (node_type start, node_type goal);

    //definition of a map node. It stores x,y information associated to a crossing cost
    class node_type
    {
        public:
        bool empty;
        int x;
        int y;
        double g_score;
        double f_score;
        double s_score;
        XYCell came_from;
        node_type();
        friend bool operator<  (const node_type &a, const node_type &b);
    };

    bool operator < (const node_type &a, const node_type &b);

    //a node_map_type contains a square matrix of w*h cells, each cell is represented by a node_type with its associated cost
    class node_map_type
    {
        public:
        node_map_type();
        node_map_type(yarp::dev::Nav2D::MapGrid2D& map);
        ~node_map_type();

        public:
        size_t w;
        size_t h;
        node_type** nodes;
    };

    class ordered_set_type
    {
        std::vector<node_type> set;

        public:
        void insert (const node_type& t);
        node_type get_smallest();
        void print();
        size_t size();
        bool find(node_type t);
    };

    class unordered_set_type
    {
        std::vector<node_type> set;

        public:
        void insert (const node_type& t);
        bool find(node_type t);
    };
};

/////// node_type
aStar_algorithm::node_type::node_type()
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

bool aStar_algorithm::operator < (const node_type &a, const node_type &b)
{
    return (a.f_score>b.f_score);
}

/////////// node_map_type
aStar_algorithm::node_map_type::node_map_type(MapGrid2D& map)
{
    w = map.width();
    h = map.height();
    nodes = new node_type* [w];
    for (int i = 0; i < w; ++i)  nodes[i] = new node_type[h];

    for (int y=0; y<h; y++)
        for (int x=0; x<w; x++)
            {
                if (map.isFree(XYCell(x, y)))
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

aStar_algorithm::node_map_type::~node_map_type()
{
    for (int i = 0; i < w; ++i) delete[] nodes[i];
    delete[] nodes;
}

/////////// ordered_set_type
void aStar_algorithm::ordered_set_type::insert (const node_type& t)
{
    set.push_back(t);
    push_heap (set.begin(),set.end()); 
}

aStar_algorithm::node_type aStar_algorithm::ordered_set_type::get_smallest()
{
    node_type t = set.front();
    pop_heap (set.begin(),set.end());
    set.pop_back();
    return t;
}
void aStar_algorithm::ordered_set_type::print()
{
    yCDebug(PATHPLAN_ASTAR,"front (smallest)%f \n", set.front().f_score);
    for (unsigned int i=0; i<set.size(); i++)
        yCDebug(PATHPLAN_ASTAR, "id%d x%d y%d %f\n", i, set[i].x, set[i].y, set[i].f_score);
    yCDebug(PATHPLAN_ASTAR, "back (biggest) %f \n", set.back().f_score);
}

size_t aStar_algorithm::ordered_set_type::size()
{
    return set.size();
}

bool aStar_algorithm::ordered_set_type::find(node_type t)
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
void aStar_algorithm::unordered_set_type::insert (const node_type& t)
{
    set.push_back(t);
}
    
bool aStar_algorithm::unordered_set_type::find(node_type t)
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
bool aStar_algorithm::find_astar_path(MapGrid2D& map, XYCell start, XYCell goal, std::deque<XYCell>& path)
{
    //implementation of A* algorithm
    std::vector<XYCell> inverse_path;
    node_map_type node_map(map);
    int sx=start.x;
    int sy=start.y;
    int gx=goal.x;
    int gy=goal.y;

    //checks that start and goal cells are inside the grid map
    if (sx>node_map.w || gx>node_map.w) return false;
    if (sy>node_map.h || gy>node_map.h) return false;
    if (sx<0  || gx<0) return false;
    if (sy<0  || gy<0) return false;

    unordered_set_type closed_set;
    ordered_set_type   open_set;  

    open_set.insert(node_map.nodes[sx][sy]);
    
    node_map.nodes[sx][sy].g_score = 0;
    node_map.nodes[sx][sy].f_score = node_map.nodes[sx][sy].g_score + heuristic_cost_estimate(node_map.nodes[sx][sy], node_map.nodes[gx][gy]);

    int iterations=0;
    while (open_set.size()>0)
    {
        iterations++;
        //yCDebug ("%d\n", iterations++);
        //open_set.print();
        node_type curr=open_set.get_smallest();
        
        if (curr.x==goal.x &&
            curr.y==goal.y) 
            {
                XYCell c;
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
                for (auto it= inverse_path.rbegin(); it!=inverse_path.rend(); it++)
                {
                    path.push_back(*it);
                }
                return true;
            }

        closed_set.insert(curr);

        //computes the list of neighbors of the current node
        list<node_type> neighbors;
        //yCDebug ("%d %d \n", curr.x, curr.y);
        if (node_map.nodes[curr.x][curr.y + 1].empty)   neighbors.push_back(node_map.nodes[curr.x][curr.y + 1]);
        if (node_map.nodes[curr.x][curr.y - 1].empty)   neighbors.push_back(node_map.nodes[curr.x][curr.y - 1]);
        if (node_map.nodes[curr.x + 1][curr.y].empty)   neighbors.push_back(node_map.nodes[curr.x + 1][curr.y]);
        if (node_map.nodes[curr.x - 1][curr.y].empty)   neighbors.push_back(node_map.nodes[curr.x - 1][curr.y]);
        if (node_map.nodes[curr.x + 1][curr.y + 1].empty) neighbors.push_back(node_map.nodes[curr.x + 1][curr.y + 1]);
        if (node_map.nodes[curr.x + 1][curr.y - 1].empty) neighbors.push_back(node_map.nodes[curr.x + 1][curr.y - 1]);
        if (node_map.nodes[curr.x - 1][curr.y + 1].empty) neighbors.push_back(node_map.nodes[curr.x - 1][curr.y + 1]);
        if (node_map.nodes[curr.x - 1][curr.y - 1].empty) neighbors.push_back(node_map.nodes[curr.x - 1][curr.y - 1]);

        //process the list of neighbors
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

double aStar_algorithm::heuristic_cost_estimate (node_type start, node_type goal)
{
    //estimate the cost from start to goal
    double dist = sqrt ( double((start.x-goal.x)*(start.x-goal.x) +
                                (start.y-goal.y)*(start.y-goal.y)))*10;
    return dist;
}

