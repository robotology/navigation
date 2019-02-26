
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Ball3DRetriever.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "TargetRetriever.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;

TargetRetriever::TargetRetriever(): m_debugOn(false){;}

bool TargetRetriever::init(std::string inputPortName, bool debugOn)
{
    if(! m_inputPort.open(inputPortName))
    {
        yError() << "TargetPointRetriever:Error opening input port";
        return false;
    }
    m_debugOn=debugOn;
    return true;
}

bool TargetRetriever::deinit(void)
{
    m_inputPort.interrupt();
    m_inputPort.close();
    return true;
}

