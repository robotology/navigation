/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file GazeController.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef GAZECONTROLLER_H
#define GAZECONTROLLER_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RpcClient.h>

namespace FollowerTarget
{
    enum class GazeCtrlLookupStates{none=0, nearest=1, otherside=2, finished=3};
    using PixelRange_t =std::pair<int, int>;
    struct pixelPoint_t
    {
        int u;
        int v;
        pixelPoint_t(int _u, int _v){u=_u; v=_v;}
    };

    //enum class GazeCtrlWellKnownPizelPoint {right=0, center=1, left=2, maxNum=3}
    enum class GazeCtrlUsedCamera {left, depth};

    class GazeController
    {
    public:
        GazeController():
                        m_lookupState(GazeCtrlLookupStates::none),
                        m_pLeft(0,0),
                        m_pCenter(0,0),
                        m_pRight(0,0),
                        xpixelRange(0,0),
                        ypixelRange(0,0),
                        m_trajectoryTime(10),
                        m_trajectoryTimeDefault(1.0)
                        {;}
        bool init(GazeCtrlUsedCamera cam, yarp::os::ResourceFinder &rf, bool debugOn=false );
        bool deinit(void);
        GazeCtrlLookupStates lookup4Target(void);
        bool stopLookup4Target(void);
        void resetLookupstateMachine(void);
        bool lookInFront(void);
        bool lookAtPixel(double u, double v);
        bool lookAtPoint(const  yarp::sig::Vector &x);
        bool lookAtAngle(double a, double b);
        bool checkMotionDone(void);
        bool lookAtPointRPC(const  yarp::sig::Vector &x);
    private:
        GazeCtrlLookupStates m_lookupState;
        yarp::os::BufferedPort<yarp::os::Property>  m_outputPort2gazeCtr; //I send commands to the gaze controller
        yarp::os::RpcClient m_rpcPort2gazeCtr;

        pixelPoint_t m_pLeft;
        pixelPoint_t m_pCenter;
        pixelPoint_t m_pRight;
        std::string m_camera_str_command;

        PixelRange_t xpixelRange;
        PixelRange_t ypixelRange;
        bool m_debugOn;
        double m_trajectoryTime;
        double m_trajectoryTimeDefault;

        bool setTrajectoryTime(double T);

    };

}

#endif
