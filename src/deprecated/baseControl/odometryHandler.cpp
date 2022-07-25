/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
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

#include "odometryHandler.h"
#include <yarp/os/LogStream.h>
#include <limits>

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0

YARP_LOG_COMPONENT(ODOM_HND, "navigation.baseControl.OdometryHandler")

OdometryHandler::~OdometryHandler()
{
    close();
}

OdometryHandler::OdometryHandler(PolyDriver* _driver)
{
    control_board_driver = _driver;
    m_odom_x               = 0;
    m_odom_y               = 0;
    m_odom_theta           = 0;
    odom_vel_x           = 0;
    odom_vel_y           = 0;
    odom_vel_lin         = 0;
    odom_vel_theta       = 0;
    base_vel_x           = 0;
    base_vel_y           = 0;
    base_vel_lin         = 0;
    base_vel_theta       = 0;
    traveled_distance    = 0;
    traveled_angle       = 0;
}

bool OdometryHandler::open(const Property &options)
{
    if (ctrl_options.check("BASECTRL_GENERAL"))
    {
        yarp::os::Bottle g_group = ctrl_options.findGroup("BASECTRL_GENERAL");
    }
    else
    {
        yCError(ODOM_HND) << "Missing [BASECTRL_GENERAL] section";
        return false;
    }

    return true;
}

void OdometryHandler::close()
{
}

double OdometryHandler::get_base_vel_lin()
{
    return this->base_vel_lin;
}

double OdometryHandler::get_base_vel_theta()
{
    return this->base_vel_theta;
}

bool OdometryHandler::getOdometry(yarp::dev::OdometryData& odomData) const
{
    data_mutex.lock();
    odomData.odom_x = m_odom_x;
    odomData.odom_y = m_odom_y;
    odomData.odom_theta = m_odom_theta;
    odomData.base_vel_x = base_vel_x;
    odomData.base_vel_y = base_vel_y;
    odomData.base_vel_theta = base_vel_theta;
    odomData.odom_vel_x = odom_vel_x;
    odomData.odom_vel_y = odom_vel_y;
    odomData.odom_vel_theta = odom_vel_theta;
    data_mutex.unlock();
    return true;
}