
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
    Target_t t; //it is initialized as invalid target

    Bottle *b = m_inputPort.read(false);
    if(nullptr == b)
    {
        //yError() <<" Ball3DPointRetriever::getTarget: I don't receive nothing!";
        return t;
    }

    bool ballIsTracked = (b->get(6).asDouble() == 1.0) ? true : false;

    if(!ballIsTracked)
    {
        if(m_debugOn)
            yDebug() << "Ball3DPointRetriever: I can't see the redBall";
        return t;
    }


    if(m_debugOn)
    {
        //yDebug() << "Ball3DPointRetriever: I see the redBall at"<< b->get(0).asDouble() << b->get(1).asDouble() << b->get(2).asDouble();
    }

    t.point3D[0] = b->get(0).asDouble();
    t.point3D[1] = b->get(1).asDouble();
    t.point3D[2] = b->get(2).asDouble();

    t.pixel[0] = b->get(4).asDouble(); //u and V are the the coordinate x any of image.
    t.pixel[1] = b->get(5).asDouble();

    t.isValid=true;

    return t;
}

