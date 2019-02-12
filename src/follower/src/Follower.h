/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file Follower.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifndef FOLLOWER_H
#define FOLLOWER_H

#include <string>
#include <mutex>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>

#include "TargetRetriver.h"
#include "SimFramePainter.h"

namespace FollowerTarget
{

    class FollowerConfig
    {
    public:
        double factorDist2Vel;
        double factorAng2Vel;
        std::string inputPortName;
        std::string outputPortName;
        double distanceThreshold = 0.8;
        double angleThreshold = 3.0;
        std::string targetType;
        struct
        {
            double angular;
            double linear;
        }velocityLimits;
        double angleMinBeforeMove;
        struct
        {
            bool enabled;
            bool paintGazeFrame;
            bool startWithoutCommand;
        }debug;

        FollowerConfig()
        {
            //init with default values
            factorDist2Vel = 0.8;
            factorAng2Vel = 0.8;
            inputPortName = "targetPoint";
            outputPortName = "commands";
            distanceThreshold = 0.8;
            angleThreshold = 3.0;
            targetType = "person";
            velocityLimits.angular = 30; //degree/sec
            velocityLimits.linear = 3; //m/s
            angleMinBeforeMove = 10.0; //degree
            debug.enabled=false;
            debug.paintGazeFrame = false;
            debug.startWithoutCommand = false;
        }

        void print(void);
    };

    enum class FollowerTargetType
    {
        redball, person
    };

    enum class FollowerStateMachine
    {
        none=0, initted=1, configured=2, running=3
    };

    class Follower
    {
    public:

        Follower();
        ~Follower();
        bool configure(yarp::os::ResourceFinder &rf);
        void followTarget(Target_t &target);
        bool start(void);
        bool stop(void);
        bool close();
        FollowerTargetType getTargetType(void);
        FollowerStateMachine getState(void);

    private:

        struct
        {
            yarp::dev::IFrameTransform* transformClient;
            yarp::dev::PolyDriver      driver;

            const std::string redBallFrameId = "head_leopard_left";
            const std::string personFrameId = "depth_center";
            const std::string baseFrameId = "mobile_base_body_link";
            std::string targetFrameId;
        }m_transformData;

        bool m_onSimulation;
        SimManager * m_simmanager_ptr;

        yarp::os::Port m_rpcPort;

        yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPort2baseCtr; //I send commands to baseControl interruptModule
        yarp::os::BufferedPort<yarp::os::Property>  m_outputPort2gazeCtr; //I send commands to the gaze controller

        FollowerConfig m_cfg;
        FollowerTargetType m_targetType;

        FollowerStateMachine m_stateMachine_st;
        Target_t m_lastValidTarget;

        std::mutex m_mutex;


        bool transformPointInBaseFrame(yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);
        bool transformPointInHeadFrame(std::string frame_src, yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);

        bool initTransformClient(void);

        bool readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg);

        bool sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity);
        bool sendCommand2GazeControl(double x, double y, double z);
        bool sendCommand2GazeControl_lookAtPixel(double u, double v);
        bool sendCommand2GazeControl_lookAtPoint(const  yarp::sig::Vector &x);
        void paintTargetPoint(const  yarp::sig::Vector &target);
        void paintTargetPoint2(yarp::sig::Vector &target);
        bool isRunningInsimulation(void) {return((m_simmanager_ptr==nullptr) ? false :true);}


        // ---- TEST STUFF
        bool moveRobot(void);
        yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPortJoystick;//test only!!!used in sendOutput
        void sendOutputLikeJoystick(); //only for test. it simulates joystick
        //get transform matrix from left camera to mobile base. Pf3dtraker use the left camera.
        bool getMatrix(yarp::sig::Matrix &transform);
    };
}
#endif

