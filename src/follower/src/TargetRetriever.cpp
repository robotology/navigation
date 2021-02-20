
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

YARP_LOG_COMPONENT(FOLLOWER_TARGETRET, "navigation.follower.targerRetriever")

TargetRetriever::TargetRetriever(): m_debugOn(false){;}

bool TargetRetriever::initInputPort(std::string inputPortName)
{
    if(! m_inputPort.open(inputPortName))
    {
        yCError(FOLLOWER_TARGETRET) << "TargetPointRetriever:Error opening input port";
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

std::string Target_tBIS::toString(void)
{
    std::string str="Target";

    if(isValid)
    {
        str+= " VALID point3d=(" + std::to_string(point3D[0]) + " " + std::to_string(point3D[1]) + " " + std::to_string(point3D[2]) + ")";
        if(hasValidPixel())
        {
            str+= " pixel=(" + std::to_string(pixel[0]) + ", " + std::to_string(pixel[1]) +")";
        }
        else
        {
            str+= " pixel NOT valid";
        }
        str+= " ref_frame=" + ReferenceFrameOfTarget2String(refFrame);
    }
    else
    {
        str+=" NOT VALID";
    }
    return str;
}




