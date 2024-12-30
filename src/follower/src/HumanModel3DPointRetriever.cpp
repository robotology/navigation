/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file Person3DPPointRetriver.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "HumanModel3DPointRetriever.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace FollowerTarget;

YARP_LOG_COMPONENT(FOLLOWER_MODELRET, "navigation.follower.modelRetriever")

bool HumanModel3DPointRetriever::init(yarp::os::ResourceFinder &rf)
{
    m_refFrame=ReferenceFrameOfTarget_t::mobile_base_body_link;

    std::string portname="/follower/humanModelRetriver/rpc";
    if(!m_worldInterfacePort.open(portname))
    {
        yCError(FOLLOWER_MODELRET) << "HumanModel3DPointRetriever: error in opening" << portname << "rpc port";
        return false;
    }


    return true;
}

Target_t HumanModel3DPointRetriever::getTarget(void)
{
    Target_t t(m_refFrame); //it is initialized to false

    // Prepare bottle containing command to send in order to get the current position
    Bottle cmdGet, ansGet;
    cmdGet.clear();
    ansGet.clear();

    cmdGet.addString("getPose");
    cmdGet.addString("Luca");
    cmdGet.addString("SIM_CER_ROBOT::mobile_base_body_link");
    m_worldInterfacePort.write(cmdGet, ansGet);

    if(m_debugOn)
        yCDebug(FOLLOWER_MODELRET) << "HumanModel3DPointRetriever: cmd-GET= " << cmdGet.toString() << "  Ans=" << ansGet.toString();


    //NOTE: if Luca doesn't exist in Gazebo than the answer is "0.0 0.0 0.0 0.0 0.0 0.0".
    //So is difficult to discriminate  the real pose containing all zeros form the case in which Luca doesn't exist,
    //but this is the only way.

    if(ansGet.size() == 0)
        return t;

    if((ansGet.get(0).asFloat64()==0) && (ansGet.get(1).asFloat64()==0) && (ansGet.get(2).asFloat64()==0)  &&
       (ansGet.get(3).asFloat64()==0)  && (ansGet.get(4).asFloat64()==0) && (ansGet.get(5).asFloat64()==0))
        return t;

    t.point3D[0] = ansGet.get(0).asFloat64();
    t.point3D[1] = ansGet.get(1).asFloat64();
    t.point3D[2] = ansGet.get(2).asFloat64();
    t.isValid=true;

    return t;
}

bool HumanModel3DPointRetriever::deinit(void)
{
    return(TargetRetriever::deinitInputPort());
}

