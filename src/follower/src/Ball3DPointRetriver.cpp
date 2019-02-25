
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Ball3DPointRetriver.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "Ball3DPointRetriver.h"

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;

Ball3DPointRetriver::Ball3DPointRetriver(): m_ballPointU(0.0), m_ballPointV(0.0) {;}


Target_t Ball3DPointRetriver::getTarget(void)
{
    std::vector<double> point3d = {0,0,0};

    Bottle *b = m_inputPort.read(false);
    if(nullptr == b)
    {
        //yError() <<" Ball3DPointRetriver::getTarget: I don't receive nothing!";
        return std::make_pair(std::move (point3d), false);
    }

    bool ballIsTracked = (b->get(6).asDouble() == 1.0) ? true : false;

    if(!ballIsTracked)
    {
        if(m_debugOn)
            yDebug() << "Ball3DPointRetriver: I can't see the redBall";
        return std::make_pair(std::move (point3d), false);
    }


    if(m_debugOn)
    {
        yDebug() << "Ball3DPointRetriver: I see the redBall at"<< b->get(0).asDouble() << b->get(1).asDouble() << b->get(2).asDouble();
    }

    point3d[0] = b->get(0).asDouble();
    point3d[1] = b->get(1).asDouble();
    point3d[2] = b->get(2).asDouble();

    m_ballPointU = b->get(4).asDouble(); //u and V are the the coordinate x any of image.
    m_ballPointV = b->get(5).asDouble();


    return std::make_pair(std::move (point3d), true);
}


void Ball3DPointRetriver::getTargetPixelCoord(double &u, double &v)
{
    u=m_ballPointU;
    v=m_ballPointV;
}
