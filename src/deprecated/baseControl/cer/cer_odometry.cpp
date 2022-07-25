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

#include "cer_odometry.h"
#include <yarp/os/LogStream.h>
#include <limits>

YARP_LOG_COMPONENT(CER_ODOM, "navigation.baseControl.cerOdometry")

bool CER_Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encL_offset);
    ienc->getEncoder(1,&encR_offset);
    m_odom_x=0;
    m_odom_y=0;
    encvel_estimator->reset();
    yCInfo(CER_ODOM,"Odometry reset done");
    return true;
}

void CER_Odometry::printStats()
{
    data_mutex.lock();
    //yCInfo (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    yCInfo(CER_ODOM,"* Odometry Thread:");
    yCInfo(CER_ODOM, "enc1:%+9.1f enc2:%+9.1f  ", enc[0]*57, enc[1]*57);
    yCInfo(CER_ODOM, "env1:%+9.3f env2:%+9.3f ", encv[0] * 57, encv[1] * 57);
    yCInfo(CER_ODOM, "ivlx:%+9.3f ivlx:%+9.3f",base_vel_lin, base_vel_theta);
    yCInfo(CER_ODOM, "ovlx:%+9.3f ovly:%+9.3f ovlt:%+9.3f", odom_vel_x, odom_vel_y, odom_vel_theta);
    yCInfo(CER_ODOM, "x: %+5.3f y: %+5.3f t: %+5.3f", m_odom_x, m_odom_y, m_odom_theta);
    data_mutex.unlock();
}

CER_Odometry::~CER_Odometry()
{
    close();
}

CER_Odometry::CER_Odometry(PolyDriver* _driver) : OdometryHandler(_driver)
{
    control_board_driver= _driver;
    m_odom_x=0;
    m_odom_y=0;
    m_odom_theta=0;

    odom_vel_x=0;
    odom_vel_y=0;
    odom_vel_lin=0;
    odom_vel_theta=0;
    base_vel_x = 0;
    base_vel_y = 0;
    base_vel_lin = 0;
    base_vel_theta = 0;

    traveled_distance=0;
    traveled_angle=0;
    encvel_estimator =new iCub::ctrl::AWLinEstimator(2,1.0);
    encw_estimator = new iCub::ctrl::AWLinEstimator(1, 1.0);
    enc.resize(2);
    encv.resize(2);
    geom_r = 0;
    geom_L = 0;
}

bool CER_Odometry::open(const Property& _options)
{
    ctrl_options = _options;

    // open the control board driver
    yCInfo(CER_ODOM,"Opening the motors interface...");

    Property control_board_options("(device remote_controlboard)");
    if (!control_board_driver)
    {
        yCError(CER_ODOM,"control board driver not ready!");
        return false;
    }
    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ienc);
    if(!ok)
    {
        yCError(CER_ODOM, "one or more devices has not been viewed");
        return false;
    }

    //reset odometry
    reset_odometry();

    //the base class open
    if (!OdometryHandler::open(_options))
    {
        yCError(CER_ODOM) << "Error in Odometry::open()"; return false;
    }

    //get robot geometry
    Bottle geometry_group = ctrl_options.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yCError(CER_ODOM, "cer_Odometry::open Unable to find ROBOT_GEOMETRY group!");
        return false;
    }
    if (!geometry_group.check("geom_r"))
    {
        yCError(CER_ODOM, "Missing param geom_r in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("geom_L"))
    {
        yCError(CER_ODOM, "Missing param geom_L in [ROBOT_GEOMETRY] group");
        return false;
    }
    geom_r = geometry_group.find("geom_r").asFloat64();
    geom_L = geometry_group.find("geom_L").asFloat64();

    return true;
}

void CER_Odometry::compute()
{
    data_mutex.lock();

    //read the encoders (deg)
    ienc->getEncoder(0,&encL);
    ienc->getEncoder(1,&encR);
        
    //read the speeds (deg/s)
    ienc->getEncoderSpeed(0,&velL);
    ienc->getEncoderSpeed(1,&velR);
        
    //remove the offset and convert in radians
    enc[0]= (encL - encL_offset) * 0.0174532925; 
    enc[1]= (encR - encR_offset) * 0.0174532925;
       
    //estimate the speeds
    iCub::ctrl::AWPolyElement el;
    el.data=enc;
    el.time=Time::now();
    encv= encvel_estimator->estimate(el);

    //compute the orientation.
    m_odom_theta = (geom_r / geom_L) * (-enc[0] + enc[1]);

    iCub::ctrl::AWPolyElement el2;
    el2.data = yarp::sig::Vector(1, m_odom_theta);
    el2.time = Time::now();
    yarp::sig::Vector vvv;
    vvv.resize(1, 0.0);
    vvv = encw_estimator->estimate(el2);

    //build the kinematics matrix
    /*yarp::sig::Matrix kin;
    kin.resize(3,2);
    kin.zero();
    kin(0, 0) = cos(odom_theta) / 2;
    kin(0, 1) = cos(odom_theta) / 2;
    kin(0, 2) = sin(odom_theta) / 2;
    kin(1, 0) = sin(odom_theta) / 2;
    kin(1, 1) = 1 / (2 * geom_L);
    kin(1, 2) = 1 / (2 * geom_L);

    yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
    yarp::sig::Vector base_cart_vels; //velocities expressed in the base reference frame
    odom_cart_vels  = kin*encv;
    base_cart_vels = kin*encv; //@@@

    base_vel_x     = base_cart_vels[1];
    base_vel_y     = base_cart_vels[0];
    base_vel_lin   = sqrt(base_vel_x*base_vel_x + base_vel_y*base_vel_y);
    base_vel_theta = base_cart_vels[2];
    
    odom_vel_x      = odom_cart_vels[1];
    odom_vel_y      = odom_cart_vels[0];
    odom_vel_theta  = odom_cart_vels[2];
    */


    base_vel_x = geom_r / 2 * encv[0] + geom_r / 2 * encv[1]; 
    base_vel_y = 0;
    base_vel_lin = fabs(base_vel_x);
    base_vel_theta = vvv[0];///-(geom_r / geom_L) * encv[0] + (geom_r / geom_L) * encv[1];
    //yCDebug() << base_vel_theta << vvv[0];

    
    odom_vel_x = base_vel_x * cos(m_odom_theta);
    odom_vel_y = base_vel_x * sin(m_odom_theta);
    odom_vel_lin = base_vel_lin;
    odom_vel_theta = base_vel_theta;

    //the integration step
    double period=el.time-last_time;
    m_odom_x= m_odom_x + (odom_vel_x * period);
    m_odom_y= m_odom_y + (odom_vel_y * period);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(base_vel_lin   * period);
    traveled_angle    = traveled_angle    + fabs(base_vel_theta * period);

    //convert from radians back to degrees
    m_odom_theta       *= RAD2DEG;
    base_vel_theta   *= RAD2DEG;
    odom_vel_theta   *= RAD2DEG;
    traveled_angle   *= RAD2DEG;

    data_mutex.unlock();
    last_time = yarp::os::Time::now();
}

