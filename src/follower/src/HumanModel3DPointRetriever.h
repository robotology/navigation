/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

