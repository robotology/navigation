
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
#include <yarp/os/ResourceFinder.h>
//#include <utility>

namespace FollowerTarget
{
    enum class ReferenceFrameOfTarget_t
    {
        head_leopard_left,
        depth_rgb,
        mobile_base_body_link
    };

    static std::string ReferenceFrameOfTarget2String(ReferenceFrameOfTarget_t refFrame)
    {
        switch(refFrame)
        {
            case ReferenceFrameOfTarget_t::head_leopard_left:return "head_leopard_left";
            case ReferenceFrameOfTarget_t::depth_rgb: return "depth_rgb";
            case ReferenceFrameOfTarget_t::mobile_base_body_link: return "mobile_base_body_link";
            default: return "";
        };
    }


    class Target_tBIS
    {
    public:
            yarp::sig::Vector point3D;
            yarp::sig::Vector pixel;
            bool isValid;
            ReferenceFrameOfTarget_t refFrame;
            Target_tBIS(ReferenceFrameOfTarget_t frame): point3D(3, 0.0), pixel(2, -1.0), isValid(false), refFrame(frame)
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
            bool hasValidPixel(){if((pixel[0]!=-1.0) && (pixel[1] != 1-.0)) return true; else return false;}
            std::string toString(void);
    };

//    using Target_t = std::pair<std::vector<double>, bool>;

    using Target_t = Target_tBIS;


    class TargetRetriever
    {
    public:
        TargetRetriever();
        virtual Target_t getTarget(void)=0;
        virtual bool init(yarp::os::ResourceFinder &rf)=0;
        virtual bool deinit(void)=0;
        void setDebug(bool on) {m_debugOn=on;}
    protected:

        bool initInputPort(std::string inputPortName);
        bool deinitInputPort(void);
        yarp::os::BufferedPort<yarp::os::Bottle> m_inputPort;
        ReferenceFrameOfTarget_t m_refFrame;
        bool m_debugOn;
    };

}
#endif
