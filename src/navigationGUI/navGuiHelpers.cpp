/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "navGui.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

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
    string s = INavigation2DHelpers::statusToString(m_navigation_status);
    return s;
}
