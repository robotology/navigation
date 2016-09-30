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

#include "ikart_odometry.h"
#include <yarp/os/LogStream.h>

bool iKart_Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encA_offset);
    ienc->getEncoder(1,&encB_offset);
    ienc->getEncoder(2,&encC_offset);
    odom_x=0;
    odom_y=0;
    encvel_estimator->reset();
    yInfo("Odometry reset done");
    return true;
}

void iKart_Odometry::printStats()
{
    mutex.wait();
    //yInfo (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    yInfo ("* Odometry Thread:");
    yInfo ("enc1:%+9.1f enc2:%+9.1f enc3:%+9.1f ******** env1:%+9.3f env2:%+9.3f env3:%+9.3f\n",
    enc[0]*57, enc[1]*57, enc[2]*57, encv[0]*57, encv[1]*57, encv[2]*57);
    
    yInfo ("ivlx:%+9.3f ivly:%+9.3f                ******** ovlx:%+9.3f ovly:%+9.3f ovlt:%+9.3f ******** x: %+5.3f y: %+5.3f t: %+5.3f\n",
    base_vel_x, base_vel_y, odom_vel_x, odom_vel_y, base_vel_theta,  odom_x, odom_y,odom_theta );
    mutex.post();
}

iKart_Odometry::~iKart_Odometry()
{
    close();
}

iKart_Odometry::iKart_Odometry(unsigned int _period, PolyDriver* _driver) : Odometry(_period, _driver)
{
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_theta=0;
    odom_vel_x=0;
    odom_vel_y=0;
    base_vel_x=0;
    base_vel_y=0;

    base_vel_lin=0;
    base_vel_theta=0;
    odom_vel_lin=0;
    traveled_distance=0;
    traveled_angle=0;
    geom_r = 0;
    geom_L = 0;
    g_angle = 0;
    encvel_estimator =new iCub::ctrl::AWLinEstimator(3,1.0);
    enc.resize(3);
    encv.resize(3);
}

bool iKart_Odometry::open(ResourceFinder &_rf, Property &_options)
{
    ctrl_options = _options;
    localName = ctrl_options.find("local").asString();

    // open the control board driver
    yInfo("Opening the motors interface...");

    Property control_board_options("(device remote_controlboard)");
    if (!control_board_driver)
    {
        yError("control board driver not ready!");
        return false;
    }
    // open the interfaces for the control boards
    bool ok = true;
    ok = ok & control_board_driver->view(ienc);
    if(!ok)
    {
        yError("one or more devices has not been viewed");
        return false;
    }
    // open control input ports
    port_odometry.open((localName+"/odometry:o").c_str());
    port_odometer.open((localName+"/odometer:o").c_str());
    port_vels.open((localName+"/velocity:o").c_str());

    //reset odometry
    reset_odometry();

    //the base class open
    if (!Odometry::open(_rf, _options))
    {
        yError() << "Error in Odometry::open()"; return false;
    }

    //yDebug() <<ctrl_options.toString();
    Bottle general_group = ctrl_options.findGroup("GENERAL");
    if (general_group.isNull())
    {
        yError("iKart_Odometry::open Unable to find GENERAL group!");
        return false;
    }

    //get robot geometry
    Bottle geometry_group = ctrl_options.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError("iKart_Odometry::open Unable to find ROBOT_GEOMETRY group!");
        return false;
    }
    if (!geometry_group.check("geom_r"))
    {
        yError("Missing param geom_r in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("geom_L"))
    {
        yError("Missing param geom_L in [ROBOT_GEOMETRY] group");
        return false;
    }
    if (!geometry_group.check("g_angle"))
    {
        yError("Missing param g_angle in [ROBOT_GEOMETRY] group");
        return false;
    }
    geom_r = geometry_group.find("geom_r").asDouble();
    geom_L = geometry_group.find("geom_L").asDouble();
    g_angle = geometry_group.find("g_angle").asDouble();

    return true;
}

void iKart_Odometry::compute()
{
    mutex.wait();

    //read the encoders (deg)
    ienc->getEncoder(0,&encA);
    ienc->getEncoder(1,&encB);
    ienc->getEncoder(2,&encC);
        
    //read the speeds (deg/s)
    ienc->getEncoderSpeed(0,&velA);
    ienc->getEncoderSpeed(1,&velB);
    ienc->getEncoderSpeed(2,&velC);
        
    //remove the offset and convert in radians
    enc[0]= -(encA - encA_offset) * 0.0174532925; 
    enc[1]= -(encB - encB_offset) * 0.0174532925;
    enc[2]= -(encC - encC_offset) * 0.0174532925;
       
    //estimate the speeds
    iCub::ctrl::AWPolyElement el;
    el.data=enc;
    el.time=Time::now();
    encv= encvel_estimator->estimate(el);

    // -------------------------------------------------------------------------------------
    // The following formulas are adapted from:
    // "A New Odometry System to reduce asymmetric Errors for Omnidirectional Mobile Robots"
    // -------------------------------------------------------------------------------------

    //compute the orientation. odom_theta is expressed in radians
    odom_theta = -geom_r*(enc[0]+enc[1]+enc[2])/(3*geom_L);

    //build the kinematics matrix
    yarp::sig::Matrix kin;
    kin.resize(3,3);
    kin.zero();
    kin(0,0) = -sqrt(3.0)/2.0;
    kin(0,1) = 0.5;
    kin(0,2) = geom_L;
    kin(1,0) = sqrt(3.0)/2.0;
    kin(1,1) = 0.5;
    kin(1,2) = geom_L;
    kin(2,0) = 0;
    kin(2,1) = -1.0;
    kin(2,2) = geom_L;
    kin      = kin/geom_r;

    yarp::sig::Matrix m_gangle;
    m_gangle.resize(3,3);
    m_gangle.zero();
    m_gangle(0,0) = cos (g_angle);
    m_gangle(0,1) = -sin (g_angle);
    m_gangle(1,0) = sin (g_angle);
    m_gangle(1,1) = cos (g_angle);
    m_gangle(2,2) = 1;

    yarp::sig::Matrix ikin = luinv(kin);
    ikin=m_gangle*ikin;

    //build the rotation matrix
    yarp::sig::Matrix m1;
    m1.resize(3,3);
    m1.zero();
    m1(0,0) = cos (odom_theta);
    m1(0,1) = -sin (odom_theta);
    m1(1,0) = sin (odom_theta);
    m1(1,1) = cos (odom_theta);
    m1(2,2) = 1;

    yarp::sig::Matrix m2;
    m2.resize(3,3);
    m2.zero();
    m2(0,0) = cos (0.0);
    m2(0,1) = -sin (0.0);
    m2(1,0) = sin (0.0);
    m2(1,1) = cos (0.0);
    m2(2,2) = 1;

    yarp::sig::Vector odom_cart_vels;  //velocities expressed in the world reference frame
    yarp::sig::Vector ikart_cart_vels; //velocities expressed in the ikart reference frame
    odom_cart_vels  = m1*ikin*encv;
    ikart_cart_vels = m2*ikin*encv;

    base_vel_x     = ikart_cart_vels[1];
    base_vel_y     = ikart_cart_vels[0];
    base_vel_theta = ikart_cart_vels[2];
    base_vel_lin   = sqrt(odom_vel_x*odom_vel_x + odom_vel_y*odom_vel_y);
    
    odom_vel_x      = odom_cart_vels[0];
    odom_vel_y      = -odom_cart_vels[1];
    odom_vel_theta  = odom_cart_vels[2];
  
    //these are not currently used
    if (base_vel_lin<0.001)
    {
        odom_vel_theta = 0;
        base_vel_theta = 0;
    }
    else
    {
        odom_vel_theta  = atan2(odom_vel_x,odom_vel_y)*RAD2DEG;
        base_vel_theta = atan2(base_vel_x,base_vel_y)*RAD2DEG;
    }

    //the integration step
    odom_x=odom_x + (odom_vel_x * period/1000.0);
    odom_y=odom_y + (odom_vel_y * period/1000.0);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(base_vel_lin   * period/1000.0);
    traveled_angle    = traveled_angle    + fabs(base_vel_theta * period/1000.0);

    /* [ -(3^(1/2)*r)/3, (3^(1/2)*r)/3,        0]
        [            r/3,           r/3, -(2*r)/3]
        [        r/(3*L),       r/(3*L),  r/(3*L)]*/

    /*odom_x = -1.73205080*geom_r/3 * encA + 1.73205080*geom_r/3 * encB;
    odom_y = geom_r/3 * encA +  geom_r/3 * encB - (2*geom_r)/3 * encC;*/
        
    /*
    odom_x = geom_r/(3* 0.86602)*
                (
                (co3p-co3m)*encA + 
                (-cos(odom_theta)-co3p)*encB + 
                (cos(odom_theta)+co3m)*encC
                );
    odom_y = geom_r/(3* 0.86602)*
                (
                (si3m+si3p)*encA + 
                (-sin(odom_theta)-si3p)*encB + 
                (sin(odom_theta)-si3m)*encC
                );
    */

    //convert from radians back to degrees
    odom_theta       *= RAD2DEG;
    base_vel_theta   *= RAD2DEG;
    odom_vel_theta   *= RAD2DEG;
    traveled_angle   *= RAD2DEG;

    mutex.post();
}

double iKart_Odometry::get_vlin_coeff()
{
    return 1;
}

double iKart_Odometry::get_vang_coeff()
{
    // theta = - r(e0 + e1 +e2)/eL
    return geom_L/geom_r;
}