
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file SimFramePainter.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef SIMPAINTERS_H
#define SIMPAINTERS_H

#include <string>
#include <memory>

#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

namespace FollowerTarget
{

    class SimFramePainter
    {
        public:
            SimFramePainter(std::string name, std::string frameRef, std::shared_ptr<yarp::os::RpcClient> worldPort, bool debugOn=false ):
            m_isCreated(false),
            m_nameOfFrame(name),
            m_worldInterfacePort_ptr(worldPort),
            m_frameIdOfRef(frameRef),
            m_debugOn(debugOn){;}
            void paint(const yarp::sig::Vector &point);
            void erase(void);
            void setDebug(bool on) {m_debugOn=on;}
        private:
            bool m_isCreated;
            std::string m_nameOfFrame;
            std::shared_ptr<yarp::os::RpcClient> m_worldInterfacePort_ptr;
            std::string m_frameIdOfRef;
            bool m_debugOn;
    };


    class SimManager
    {
        public:
            bool init(std::string robotName, std::string rpcNamePort, bool debugOn);
            bool deinit(void);
            void PaintGazeFrame(const yarp::sig::Vector &point);
            void PaintTargetFrame(const yarp::sig::Vector &point);

        private:
            std::shared_ptr<yarp::os::RpcClient> m_worldInterfacePort_ptr;
            std::unique_ptr<SimFramePainter> gazeFramePainter_ptr;
            std::unique_ptr<SimFramePainter> targetFramePainter_ptr;
    };

}
//NOTE: What happen if I try to use a not opened port? I need a status variable?? (TODO check)

#endif
