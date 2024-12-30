/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file Ball3DPointRetriever.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef BALL3DPOINTRETRIVER_H
#define BALL3DPOINTRETRIVER_H


#include "TargetRetriever.h"

namespace FollowerTarget
{
    class Ball3DPointRetriever : public TargetRetriever
    {
    public:
        Target_t getTarget(void);
        bool init(yarp::os::ResourceFinder &rf);
        bool deinit(void);

    };
}
#endif
