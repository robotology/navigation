/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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

#include "baseControl.h"

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

YARP_LOG_COMPONENT(BASECONTROL_MAIN, "navigation.baseControl.main")

/////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    //Initialize Yarp network
    Network yarp;

    if (!yarp.checkNetwork())
    {
        std::cerr << "Sorry YARP network does not seem to be available, is the yarp server available?\n";
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("navigation");
    rf.setDefaultConfigFile("baseCtrl.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yCInfo(BASECONTROL_MAIN,"Possible options: ");
        yCInfo(BASECONTROL_MAIN, "'rate <r>' sets the threads rate (default 20ms).");
        yCInfo(BASECONTROL_MAIN, "'no_filter' disables command filtering.");
        yCInfo(BASECONTROL_MAIN, "'no_motors' motor interface will not be opened.");
        yCInfo(BASECONTROL_MAIN, "'no_start' do not automatically enables pwm.");
        yCInfo(BASECONTROL_MAIN, "'joystick_connect' tries to automatically connect to the joystickCtrl output.");
        yCInfo(BASECONTROL_MAIN, "'skip_robot_interface_check' does not connect to robotInterface/rpc (useful for simulator)");
        return 0;
    }

    //Starts the control module
    CtrlModule mod;
    return mod.runModule(rf);
}
