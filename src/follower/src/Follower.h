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
#include "GazeController.h"
#include "NavigationController.h"

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
        int invalidTargetMax;
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
        bool onSimulator;

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
            invalidTargetMax = 10;
            onSimulator=true;
        }

        void print(void);
    };

    enum class TargetType_t
    {
        redball, person
    };

    enum class StateMachine
    {
        none       = 0, //at creation time or if configuration fails
        configured = 1, //if configuration has success or stop received
        running    = 2, //after start(tick) received
    };

    enum class RunningSubStMachine //these are the states of sub state machine when it is in running
    {
        targetValid              = 0,
        maybeLostTarget          = 1,
        lostTarget_lookup        = 2,
        startAutoNav             = 3,
        waitAutoNav              = 4,
        autoNavOk                = 5,
        autoNavError             = 6,
        needHelp                 = 7
    };

    enum class Result_t
    {
        ok,
        notRunning,
        lostTarget,
        autoNavigation,
        error,
        needHelp
    };


    class Follower
    {
    public:

        Follower();
        ~Follower();
        bool configure(yarp::os::ResourceFinder &rf);
        Result_t followTarget(Target_t &target);
        bool start(void);
        bool stop(void);
        bool close(); // Close function, to perform cleanup.
        TargetType_t getTargetType(void);
        StateMachine getState(void);

        bool helpProvided(void); //for test purpose

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


        yarp::os::Port m_rpcPort;
        yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPort2baseCtr; //I send commands to baseControl interruptModule

        FollowerConfig m_cfg;
        TargetType_t m_targetType;

        StateMachine m_stateMachine_st;
        RunningSubStMachine m_runStMachine_st;
        yarp::sig::Vector m_lastValidPoint; //respect mobile_base_body_link frame here I save the point of m_lastValidTarget stransformed respect mobile_base_body_link
        Target_t m_lastValidTarget;
        uint32_t m_lostTargetcounter;
        bool m_autoNavAlreadyDone;

        std::mutex m_mutex;

        GazeController m_gazeCtrl;
        NavigationController m_navCtrl;
        SimManager * m_simmanager_ptr;

        bool transformPointInBaseFrame(yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);
        bool transformPointInHeadFrame(std::string frame_src, yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput);

        bool initTransformClient(void);

        bool readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg);

        bool sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity);

        bool isOnSimulator(void) {return((m_simmanager_ptr==nullptr) ? false :true);}

        bool isInRunningState(void);
        Result_t processTarget(Target_t &target);
        void goto_targetValid_state();

        // ---- TEST STUFF
        bool moveRobot(void);
        yarp::os::BufferedPort<yarp::os::Bottle>  m_outputPortJoystick;//test only!!!used in sendOutput
        void sendOutputLikeJoystick(); //only for test. it simulates joystick
        //get transform matrix from left camera to mobile base. Pf3dtraker use the left camera.
        bool getMatrix(yarp::sig::Matrix &transform);
    };
}
#endif

