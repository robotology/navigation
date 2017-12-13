/*
* Copyright (C)2017  iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Node.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class Controller
{
private:
    //robot description
    bool                      m_is_holonomic;

    //TF device driver
    PolyDriver                m_ptf;
    IFrameTransform*          m_iTf;

    //robot position
    double                    m_current_theta;
    double                    m_current_x;
    double                    m_current_y;

    //YARP ports
    yarp::os::Stamp           m_timeStamp;
    BufferedPort<Bottle>      m_port_odometry;
    BufferedPort<Bottle>      m_port_odometer;

    //variables to control odometry simulated errors
    double              odometry_error_x_gain;
    double              odometry_error_y_gain;
    double              odometry_error_t_gain;

public:
    /**
    * Default Constructor
    */
    Controller();
    
    /**
    * Destructor
    */
    ~Controller();
    
    /**
    * Performs the control action, moving the robot
    * @param lin_spd robot linear speed, expressed in m/s
    * @param ang_spd robot angular speed, expressed in deg/s 
    * @param des_dir robot heading, expressed in deg w.r.t robot local reference frame
    * @param pwm_gain the amount of power given to the simulated motors (0-100)
    */
    void   apply_control(double& lin_spd , double& ang_spd, double& des_dir, double& pwm_gain);

    /**
    * Gets robot odometry data
    * @param x robot pose (m)
    * @param y robot pose (m)
    * @param t robot pose (deg)
    */
    void   get_odometry(double& x, double& y, double& theta);
    
    /**
    * Initializes robot odometry data
    * @param x robot pose (m)
    * @param y robot pose (m)
    * @param t robot pose (deg)
    */
    void   reset(double x = 0, double y = 0, double t = 0);
    
    /**
    * Publish robot odometry via tf server
    */
    void   publish_tf();

    /**
    * Publish robot odometry data via yarp port
    */
    void   publish_port();
    
    /**
    * Initialize the system
    * @param holonomic if true, a fully holonomic model is used. Otherwise robot velocity along y axis is constrained to be zero.
    */
    bool   init(bool holonomic);
    
    /**
    * Sets x,y,t gains to simulate a systemic error, similar to wheels slippage, during odometry computation.
    * @param gain on x axis. A value of 1.0 means no error.
    * @param gain on y axis. A value of 1.0 means no error.
    * @param gain on robot heading. A value of 1.0 means no error.
    */
    void   set_odometry_error (double xgain, double ygain, double tgain);
};

#endif
