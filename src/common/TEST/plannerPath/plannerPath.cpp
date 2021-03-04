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

#include "PlannerPath.h"

using namespace yarp::dev;

size_t PlannerPath::size()
{
    return waypoints.size();
}
void PlannerPath::pop()
{
    waypoints.pop_front();
}

void PlannerPath::current_waypoint(yarp::dev::MapGrid2D::XYCell& wayp)
{
    wayp = waypoints.front().waypoint_cell;
}

void PlannerPath::clear()
{
    waypoints.clear();
    //final_goal.clear();
}

void PlannerPath::print()
{
    auto it = waypoints.begin();
    for (int counter = 0; it != waypoints.end(); it++, counter++)
    {
      //  double xw = m_current_map.cell2World(*it).x;
     //   double yw = m_current_map.cell2World(*it).y;
      //  yDebug() << counter << " x:" << xw << " y:" << yw << "(" << it->x << " " << it->y << ")";
    }
}

void PlannerPath::push(yarp::dev::Map2DLocation wayp)
{
    waypoint_t tmp;
    tmp.
    waypoints.push_back(tmp);
}

void PlannerPath::push(yarp::dev::MapGrid2D::XYCell  wayp)
{
    waypoint_t tmp;
    tmp.
    waypoints.push_back(tmp);
}

void PlannerPath::push(waypoint_t  wayp)
{
    waypoints.push_back(wayp);
}

PlannerPath::PlannerPath(yarp::dev::MapGrid2D::MapInfo info)
{
    map_info = info;
}

bool PlannerPath::getPath(std::vector<yarp::dev::Map2DLocation>& path)
{
    for (auto it= waypoints.begin(); it!=waypoints.end(); it++)
    {
        MapGrid2D::XYCell cell = *it;
        Map2DLocation loc;
        loc.map_id = this->map_info..getMapName();
        MapGrid2D::XYWorld w = m_current_map.cell2World(cell);
        loc.x = w.x;
        loc.y = w.y;
        loc.theta = 0;
        path.push_back(loc);
    }
    path.push_back(final_goal);
}