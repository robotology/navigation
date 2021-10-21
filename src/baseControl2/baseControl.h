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
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "controlThread.h"

/**
 * \section baseControl
 * This core module controls the robot joints in order to achieve the desired cartesian velocity command, sent either via a joystick or via YARP port/ROS topic
 * The module requires the definition of the robot kinematics. So far only ikart_V1, ikart_V2, cer types are valid options.as robot_type parameter.
 * Four control modes are available: velocity_no_pid sets the control modes of the motors to velocity and sends individual velocity references, computed by from user cartesian commands, to the joints.
 * velocity_pid also controls the motors in velocity modes, but it also implements and external closed loop on the realized cartesian trajectory.
 * openloop_no_pid/openloop_pid are similar, but the individual motors are controlled by sending pwm references instead of velocity commands.
 * The module also performs the odometry computation and publishes it onto a yarp port/ROS topic.
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr=nullptr;
    Port            rpcPort;
    bool            verbose_print=true;

public:
    CtrlModule() {};
    bool   configure(ResourceFinder &rf) override;
    bool   respond(const Bottle& command, Bottle& reply);
    bool   close();
    double getPeriod();
    bool   updateModule();
};
