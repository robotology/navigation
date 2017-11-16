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

#include "sim_odometry.h"
#include <yarp/os/LogStream.h>
#include <limits>

bool SIM_Odometry::reset_odometry()
{
    ienc->getEncoder(0,&encL_offset);
    ienc->getEncoder(1,&encR_offset);
    odom_x=0;
    odom_y=0;
    encvel_estimator->reset();
    yInfo("Odometry reset done");
    return true;
}

void SIM_Odometry::printStats()
{
    mutex.wait();
    //yInfo (stdout,"Odometry Thread: Curr motor velocities: %+3.3f %+3.3f %+3.3f\n", velA, velB, velC);
    yInfo("* Odometry Thread:");
    yInfo("enc1:%+9.1f enc2:%+9.1f  ", enc[0]*57, enc[1]*57);
    yInfo("env1:%+9.3f env2:%+9.3f ", encv[0] * 57, encv[1] * 57);
    yInfo("ivlx:%+9.3f ivlx:%+9.3f",base_vel_lin, base_vel_theta);
    yInfo("ovlx:%+9.3f ovly:%+9.3f ovlt:%+9.3f", odom_vel_x, odom_vel_y, odom_vel_theta);
    yInfo("x: %+5.3f y: %+5.3f t: %+5.3f", odom_x, odom_y, odom_theta);
    mutex.post();
}

SIM_Odometry::~SIM_Odometry()
{
    close();
}

SIM_Odometry::SIM_Odometry(unsigned int _period, PolyDriver* _driver) : Odometry(_period, _driver)
{
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_theta=0;

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
    rosMsgCounter=0;
    geom_r = 0;
    geom_L = 0;
}

bool SIM_Odometry::open(ResourceFinder &_rf, Property& _options)
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
    bool ret = true;
    ret &= port_odometry.open((localName+"/odometry:o").c_str());
    ret &= port_odometer.open((localName+"/odometer:o").c_str());
    ret &= port_vels.open((localName+"/velocity:o").c_str());
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }

    //reset odometry
    reset_odometry();

    //the base class open
    if (!Odometry::open(_rf, _options))
    {
        yError() << "Error in Odometry::open()"; return false;
    }

    //get robot geometry
    Bottle geometry_group = ctrl_options.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError("cer_Odometry::open Unable to find ROBOT_GEOMETRY group!");
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
    geom_r = geometry_group.find("geom_r").asDouble();
    geom_L = geometry_group.find("geom_L").asDouble();

    return true;
}

void SIM_Odometry::compute()
{
    mutex.wait();

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
    odom_theta = (geom_r / geom_L) * (-enc[0] + enc[1]);

    iCub::ctrl::AWPolyElement el2;
    el2.data = yarp::sig::Vector(1,odom_theta);
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
    //yDebug() << base_vel_theta << vvv[0];

    
    odom_vel_x = base_vel_x * cos(odom_theta);
    odom_vel_y = base_vel_x * sin(odom_theta);
    odom_vel_lin = base_vel_lin;
    odom_vel_theta = base_vel_theta;

    //the integration step
    odom_x=odom_x + (odom_vel_x * period/1000.0);
    odom_y=odom_y + (odom_vel_y * period/1000.0);

    //compute traveled distance (odometer)
    traveled_distance = traveled_distance + fabs(base_vel_lin   * period/1000.0);
    traveled_angle    = traveled_angle    + fabs(base_vel_theta * period/1000.0);

    //convert from radians back to degrees
    odom_theta       *= RAD2DEG;
    base_vel_theta   *= RAD2DEG;
    odom_vel_theta   *= RAD2DEG;
    traveled_angle   *= RAD2DEG;

    mutex.post();
}

double SIM_Odometry::get_vlin_coeff()
{
    return 1 / geom_r;
}

double SIM_Odometry::get_vang_coeff()
{
    return geom_L / (2 * geom_r);
}
