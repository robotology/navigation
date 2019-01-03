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
    string s = yarp::dev::NavigationStatusEnumHelpers::statusToString(m_navigation_status);
    return s;
}
