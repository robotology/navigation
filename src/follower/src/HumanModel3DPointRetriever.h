/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/
/**
 * @file HumanModel3DPointRetriever.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */
#ifndef HUMANMODEL3DPOINTRESTRIVER_H
#define HUMANMODEL3DPOINTRESTRIVER_H

#include "TargetRetriever.h"

#include <yarp/os/RpcClient.h>

namespace FollowerTarget
{
    class HumanModel3DPointRetriever : public TargetRetriever
    {
    public:
        Target_t getTarget(void);
        bool init(yarp::os::ResourceFinder &rf);
        bool deinit(void);
    private:
        yarp::os::RpcClient m_worldInterfacePort;

    };
}

#endif

