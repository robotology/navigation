
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Ball3DPointRetriever.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "Ball3DPointRetriever.h"

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;


Target_t Ball3DPointRetriever::getTarget(void)
{
    Target_t t(m_refFrame); //it is initialized as invalid target

    Bottle *b = m_inputPort.read(false);
    if(nullptr == b)
    {
//         yError() <<" Ball3DPointRetriever::getTarget: I don't receive nothing!";
        return t;
    }

    bool ballIsTracked = (b->get(6).asDouble() == 1.0) ? true : false;

    if(!ballIsTracked)
    {
//         if(m_debugOn)
//             yDebug() << "Ball3DPointRetriever: I can't see the redBall";
        return t;
    }


//     if(m_debugOn)
//     {
//         yDebug() << "Ball3DPointRetriever: I see the redBall at"<< b->get(0).asDouble() << b->get(1).asDouble() << b->get(2).asDouble();
//     }

    t.point3D[0] = b->get(0).asDouble();
    t.point3D[1] = b->get(1).asDouble();
    t.point3D[2] = b->get(2).asDouble();

    t.pixel[0] = b->get(4).asDouble(); //u and V are the the coordinate x any of image.
    t.pixel[1] = b->get(5).asDouble();

    t.isValid=true;

    return t;
}

bool Ball3DPointRetriever::init(yarp::os::ResourceFinder &rf)
{
    // 1) set my reference frame
    m_refFrame=ReferenceFrameOfTarget_t::head_leopard_left;

    // 2) read name of input port from config file and open it
    std::string inputPortName="targets";
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yError() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("inputPort"))  {inputPortName = config_group.find("inputPort").asString(); }
    }

    bool ret = TargetRetriever::initInputPort("/follower/" + inputPortName +":i");

    return ret;
}


bool Ball3DPointRetriever::deinit(void)
{
    return(TargetRetriever::deinitInputPort());
}

