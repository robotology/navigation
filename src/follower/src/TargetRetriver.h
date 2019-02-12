
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file TargetRetriver.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef TARGETRETRIVER_H
#define TARGETRETRIVER_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <vector>
//#include <utility>

namespace FollowerTarget
{
    using Target_t = std::pair<std::vector<double>, bool>;

    class TargetRetriver
    {
    public:
        TargetRetriver();
        virtual Target_t getTarget(void)=0;
        bool init(std::string inputPortName, bool debugOn=false);
        bool deinit(void);
    protected:
        std::vector<double> m_target;
        yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort;

        bool m_debugOn;
    };

}
#endif
