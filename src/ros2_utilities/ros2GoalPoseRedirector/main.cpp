/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <yarp/os/Network.h>
#include "Ros2GoalPoseRedirector.h"

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("redirector_def.ini");           //overridden by --from parameter
    rf.setDefaultContext("ros2GoalPoseRedirector");          //overridden by --context parameter
    rf.configure(argc,argv);
    //std::string debug_rf = rf.toString();
    Ros2GoalPoseRedirector redirector;

    return redirector.runModule(rf);
}
