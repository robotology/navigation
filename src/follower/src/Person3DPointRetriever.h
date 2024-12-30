/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file Person3DPointRetriver.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */
#ifndef PERSON3DPOINTRESTRIVER_H
#define PERSON3DPOINTRESTRIVER_H

#include "TargetRetriever.h"

#include "AssistiveRehab/skeleton.h"

namespace FollowerTarget
{
    class Person3DPointRetriever : public TargetRetriever
    {
    public:
        Target_t getTarget(void);
        bool init(yarp::os::ResourceFinder &rf);
        bool deinit(void);
    private:
        assistive_rehab::SkeletonStd m_sk_target;
    };
}

#endif

