
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Ball3DRetriver.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "TargetRetriver.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;

TargetRetriver::TargetRetriver(): m_debugOn(false){;}

bool TargetRetriver::init(std::string inputPortName, bool debugOn)
{
    if(! m_inputPort.open(inputPortName))
    {
        yError() << "TargetPointRetriver:Error opening input port";
        return false;
    }
    m_debugOn=debugOn;
    return true;
}

bool TargetRetriver::deinit(void)
{
    m_inputPort.interrupt();
    m_inputPort.close();
    return true;
}

