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

#include "navGui.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

NavigationStatusEnum NavGuiThread::string2status(string s)
{
    NavigationStatusEnum status;
    if (s == "navigation_status_idle")     status = navigation_status_idle;
    else if (s == "navigation_status_moving")   status = navigation_status_moving;
    else if (s == "navigation_status_waiting_obstacle")  status = navigation_status_waiting_obstacle;
    else if (s == "navigation_status_goal_reached")  status = navigation_status_goal_reached;
    else if (s == "navigation_status_aborted")  status = navigation_status_aborted;
    else if (s == "navigation_status_failing")  status = navigation_status_failing;
    else if (s == "navigation_status_paused")   status = navigation_status_paused;
    else if (s == "navigation_status_preparing_before_move")   status = navigation_status_preparing_before_move;
    else if (s == "navigation_status_thinking") status = navigation_status_thinking;
    else if (s == "navigation_status_error") status = navigation_status_error;
    else 
    {
        status = navigation_status_error;
    }
    return status;
}

std::string NavGuiThread::getStatusAsString(NavigationStatusEnum status)
{
    if (status == navigation_status_idle) return std::string("navigation_status_idle");
    else if (status == navigation_status_moving) return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle) return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached) return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted) return std::string("navigation_status_aborted");
    else if (status == navigation_status_failing) return std::string("navigation_status_failing");
    else if (status == navigation_status_paused) return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move) return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking) return std::string("navigation_status_thinking");
    else if (status == navigation_status_error) return std::string("navigation_status_error");
    return std::string("navigation_status_error");
}

void  NavGuiThread::getTimeouts(int& localiz, int& laser, int& nav_status)
{
    localiz = m_loc_timeout_counter;
    laser = m_laser_timeout_counter;
    nav_status = m_nav_status_timeout_counter;
}

void NavGuiThread::printStats()
{
    //To be implemented
}

NavigationStatusEnum NavGuiThread::getNavigationStatusAsInt()
{
    return m_navigation_status;
}

string NavGuiThread::getNavigationStatusAsString()
{
    string s = getStatusAsString(m_navigation_status);
    return s;
}
