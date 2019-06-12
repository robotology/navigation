/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file main.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "FollowerModule.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>



using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    /* create your module */
    FollowerTarget::FollowerModule module;
    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.configure(argc, argv);
    rf.setVerbose(true);
    yDebug() << "Configuring and starting module. \n";
    module.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    yDebug()<<"Main returning...";
    return 0;
}
