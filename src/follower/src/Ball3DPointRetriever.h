
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

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
        Ball3DPointRetriever();
        void getTargetPixelCoord(double &u, double &v);
    private:

        double m_ballPointU;
        double m_ballPointV;
    };
}
#endif
