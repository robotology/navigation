/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#define _USE_MATH_DEFINES
#include "localization_device_with_estimated_odometry.h"
#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef RAD2DEG
#define RAD2DEG 180/M_PI
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif


//////////////////////////
localization_device_with_estimated_odometry::localization_device_with_estimated_odometry()
{
    m_estimator = new iCub::ctrl::AWLinEstimator(3, 3);
}

localization_device_with_estimated_odometry::~localization_device_with_estimated_odometry()
{
    delete m_estimator;
    m_estimator=nullptr;
}

yarp::dev::OdometryData localization_device_with_estimated_odometry::estimate(const yarp::dev::Nav2D::Map2DLocation& m_localization_data)
{
    m_current_odom_mutex.lock();
    //m_current_loc is in the world reference frame.
    // hence this velocity is estimated in the world reference frame.
    iCub::ctrl::AWPolyElement el;
    el.data = yarp::sig::Vector(3);
    el.data[0] = m_current_odom.odom_x = m_localization_data.x;
    el.data[1] = m_current_odom.odom_y = m_localization_data.y;
    el.data[2] = m_current_odom.odom_theta = m_localization_data.theta;
    el.time = Time::now();
    m_odom_vel.resize(3, 0.0);
    m_odom_vel = m_estimator->estimate(el);
    m_current_odom.odom_vel_x = m_odom_vel[0];
    m_current_odom.odom_vel_y = m_odom_vel[1];
    m_current_odom.odom_vel_theta = m_odom_vel[2];

    //this is the velocity in robot reference frame.
    //NB: for a non-holonomic robot robot_vel[1] ~= 0
    m_robot_vel.resize(3, 0.0);
    m_robot_vel[0] = m_odom_vel[0] * cos(m_localization_data.theta * DEG2RAD) - m_odom_vel[0] * sin(m_localization_data.theta * DEG2RAD);
    m_robot_vel[1] = m_odom_vel[1] * sin(m_localization_data.theta * DEG2RAD) + m_odom_vel[1] * cos(m_localization_data.theta * DEG2RAD);
    m_robot_vel[2] = m_odom_vel[2];
    m_current_odom.base_vel_x = m_robot_vel[0];
    m_current_odom.base_vel_y = m_robot_vel[1];
    m_current_odom.base_vel_theta = m_robot_vel[2];

    m_current_odom_mutex.unlock();
    return m_current_odom;
}

yarp::dev::OdometryData localization_device_with_estimated_odometry::getOdometry()
{
    return m_current_odom;
}