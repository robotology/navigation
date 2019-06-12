/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file Person3DPPointRetriver.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "Person3DPointRetriever.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace assistive_rehab;
using namespace FollowerTarget;

Target_t Person3DPointRetriever::getTarget(void)
{
    Target_t t(m_refFrame); //it is initialized as not valid target

    Bottle *b = m_inputPort.read(false); //use false in order to make the reading not blocking
    if(nullptr == b)
    {
        if(m_debugOn)
            yDebug() <<" Person3DPointRetriever::getTarget: I received nothing!";

        return t;
    }

    Bottle *b1=b->get(0).asList();
    if(nullptr == b1)
    {
        if(m_debugOn)
            yDebug() << "Person3DPointRetriever: I can't read the bottle";

        return t;
    }

    if (b1->check("tag"))
    {
        Property prop(b1->toString().c_str());
        m_sk_target.fromProperty(prop);
        if(m_debugOn)
            yDebug() << "Person3DPPointRetriver: skeleton is updated!";
    }
    else
    {
        if(m_debugOn)
            yDebug() << "Person3DPPointRetriver: tag not exist!";
        return t;
    }


    const KeyPoint* targetPoint_ptr = m_sk_target[KeyPointTag::shoulder_center];
    if(targetPoint_ptr != nullptr)//is necessary this check??? maybe yes if I can't see the shoulders
    {
        if(targetPoint_ptr->isUpdated())
        {
            t.point3D=targetPoint_ptr->getPoint();
            t.pixel = targetPoint_ptr->getPixel();
            t.isValid=true;
            if(m_debugOn)
               yDebug() << "Person3DPPointRetriver: get the point!! OK!! TAG=" << m_sk_target.getTag();
        }
        else
        {
            if(m_debugOn)
                yDebug() << "Person3DPPointRetriver: the skeleton is not updated!";
        }
    }
    else
    {
        if(m_debugOn)
            yDebug() << "Person3DPPointRetriver: shoulder_center point is null!";
    }

    return t;
}

bool Person3DPointRetriever::init(yarp::os::ResourceFinder &rf)
{
    // 1) set my reference frame
    m_refFrame=ReferenceFrameOfTarget_t::depth_rgb;

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


bool Person3DPointRetriever::deinit(void)
{
    return(TargetRetriever::deinitInputPort());
}
