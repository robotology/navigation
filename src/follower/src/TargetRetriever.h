
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file TargetRetriever.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef TARGETRETRIEVER_H
#define TARGETRETRIEVER_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <vector>
//#include <utility>

namespace FollowerTarget
{
    class Target_tBIS
    {
    public:
            yarp::sig::Vector point3D;
            yarp::sig::Vector pixel;
            bool isValid;
            Target_tBIS(): point3D(3, 0.0), pixel(2, 0.0), isValid(false)
            {;};

            ~Target_tBIS()=default;
            Target_tBIS(const Target_tBIS &other)=default;
            //TODO: commented because the yarp::sig::Vector hasn't movable constructor
//             Target_tBIS(Target_tBIS &&other) noexcept
//             {
//                 point3D(std::move(other.point3D));
//                 pixel(std::move(other.pixel));
//                 isValid=other.isValid;
//             };

            Target_tBIS& operator=(const Target_tBIS &other)=default;
//             //TODO: commented because the yarp::sig::Vector hasn't movable operator=
//             Target_tBIS& operator=(Target_tBIS &&other) noexcept
//             {
//                 point3D=(std::move(other.point3D));
//                 pixel=(std::move(other.pixel));
//                 isValid=other.isValid;
//                 return *this;
//             };

    };

//    using Target_t = std::pair<std::vector<double>, bool>;

    using Target_t = Target_tBIS;


    class TargetRetriever
    {
    public:
        TargetRetriever();
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
