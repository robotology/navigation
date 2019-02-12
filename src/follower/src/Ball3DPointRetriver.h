
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Ball3DPointRetriver.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef BALL3DPOINTRETRIVER_H
#define BALL3DPOINTRETRIVER_H


#include "TargetRetriver.h"

namespace FollowerTarget
{
    class Ball3DPointRetriver : public TargetRetriver
    {
    public:
        Target_t getTarget(void);
        Ball3DPointRetriver();
        void getTargetPixelCoord(double &u, double &v);
    private:

        double m_ballPointU;
        double m_ballPointV;
    };
}
#endif
