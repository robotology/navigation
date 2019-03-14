
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file TargetRetriever.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "TargetRetriever.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;

TargetRetriever::TargetRetriever(): m_debugOn(false){;}

bool TargetRetriever::initInputPort(std::string inputPortName)
{
    if(! m_inputPort.open(inputPortName))
    {
        yError() << "TargetPointRetriever:Error opening input port";
        return false;
    }
    return true;
}

bool TargetRetriever::deinitInputPort(void)
{
    m_inputPort.interrupt();
    m_inputPort.close();
    return true;
}

