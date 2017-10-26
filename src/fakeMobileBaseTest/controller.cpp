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

#include "controller.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define DEG2RAD 3.14/180.0

Controller::Controller()
{

}

bool Controller::init(bool holonomic)
{
    m_is_holonomic = holonomic;
    reset();
    m_iTf = 0;

    Property options;
    options.put("device", "transformClient");
    options.put("local", "/fakeMobileBaseTest/localizationTfClient");
    options.put("remote", "/transformServer");
    if (m_ptf.open(options) == false)
    {
        yError() << "Unable to open transform client";
        return false;
    }
    m_ptf.view(m_iTf);
    if (m_ptf.isValid() == false || m_iTf == 0)
    {
        yError() << "Unable to view iTransform interface";
        return false;
    }

    m_port_odometry.open("/fakeMobileBaseTest/odometry:o");
    m_port_odometer.open("/fakeMobileBaseTest/odometer:o");
    return true;
}

Controller::~Controller()
{
    m_ptf.close();
    m_port_odometry.close();
    m_port_odometer.close();
}

void  Controller::publish_tf()
{
    yarp::sig::Matrix m1;
    m1.resize(4, 4);
    double a = m_current_theta*DEG2RAD;
    m1[0][0] = cos(a);   m1[0][1] = -sin(a);  m1[0][2] = 0; m1[0][3] = m_current_x;
    m1[1][0] = sin(a);   m1[1][1] = cos(a);   m1[1][2] = 0; m1[1][3] = m_current_y;
    m1[2][0] = 0;        m1[2][1] = 0;        m1[2][2] = 1; m1[2][3] = 0;
    m1[3][0] = 0;        m1[3][1] = 0;        m1[3][2] = 0; m1[3][3] = 1;
    m_iTf->setTransform("mobile_base_body", "map", m1);
}

void  Controller::publish_port()
{
    m_timeStamp.update();
    if (m_port_odometry.getOutputCount()>0)
    {
        m_port_odometry.setEnvelope(m_timeStamp);
        Bottle &b = m_port_odometry.prepare();
        b.clear();
        b.addDouble(m_current_x); //position in the odom reference frame
        b.addDouble(m_current_y);
        b.addDouble(m_current_theta);
        b.addDouble(0); //velocity in the robot reference frame
        b.addDouble(0);
        b.addDouble(0);
        b.addDouble(0); //velocity in the odom reference frame
        b.addDouble(0);
        b.addDouble(0);
        m_port_odometry.write();
    }

    if (m_port_odometer.getOutputCount()>0)
    {
        m_port_odometer.setEnvelope(m_timeStamp);
        Bottle &t = m_port_odometer.prepare();
        t.clear();
        t.addDouble(0);
        t.addDouble(0);
        m_port_odometer.write();
    }
}

void  Controller::apply_control(double& lin_spd, double& ang_spd, double& des_dir, double& pwm_gain)
{
    double dt = 0.01;
    m_current_theta = m_current_theta + ang_spd*dt;
    m_current_x = m_current_x + lin_spd*dt*cos((des_dir + m_current_theta)*DEG2RAD);
    if (m_is_holonomic)
    {
        m_current_y = m_current_y + lin_spd*dt*sin((des_dir + m_current_theta)*DEG2RAD);
    }
    else
    {
        //do nothing
        m_current_y = m_current_y;
    }
}

void  Controller::get_odometry(double& x, double& y, double& theta)
{
    x = m_current_x;
    y = m_current_y;
    theta = m_current_theta;
}

void Controller::reset( double x , double y, double t)
{
    m_current_theta = t;
    m_current_x = x;
    m_current_y = y;
}

