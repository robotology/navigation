/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file Follower.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <cmath>

#include "Follower.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace FollowerTarget;

YARP_LOG_COMPONENT(FOLLOWER, "navigation.follower")

void FollowerConfig::print(void)
{
    yCInfo(FOLLOWER) << "The follower module has been configure with following values";
    yCInfo(FOLLOWER) << "inputPortName="       << inputPortName;
    yCInfo(FOLLOWER) << "factorDist2Vel="      << outputPortName;
    yCInfo(FOLLOWER) << "targetType="          << targetType;
    yCInfo(FOLLOWER) << "invalidTargetMax="    << invalidTargetMax;
    yCInfo(FOLLOWER) << "startWithoutCommand=" << startWithoutCommand;
    yCInfo(FOLLOWER) << "onSimulator="         << onSimulator;
    yCInfo(FOLLOWER) << "NAVIGATION.factorDist2Vel="    << navigation.factorDist2Vel;
    yCInfo(FOLLOWER) << "NAVIGATION.factorAng2Vel="     << navigation.factorAng2Vel;
    yCInfo(FOLLOWER) << "NAVIGATION.distanceThreshold=" << navigation.distanceThreshold;
    yCInfo(FOLLOWER) << "NAVIGATION.angleThreshold="    << navigation.angleThreshold;
    yCInfo(FOLLOWER) << "NAVIGATION.angularVelLimit="   << navigation.velocityLimits.angular;
    yCInfo(FOLLOWER) << "NAVIGATION.linearVelLimit="    << navigation.velocityLimits.linear;
    yCInfo(FOLLOWER) << "NAVIGATION.angleLimitsVelReduction.min="<< navigation.angleLimitsVelReduction.min;
    yCInfo(FOLLOWER) << "NAVIGATION.angleLimitsVelReduction.max="<< navigation.angleLimitsVelReduction.max;
    yCInfo(FOLLOWER) << "DEBUG.enabled="        << debug.enabled;
    yCInfo(FOLLOWER) << "DEBUG.paintGazeFrame=" << debug.paintGazeFrame;
    yCInfo(FOLLOWER) << "DEBUG.printPeriod=" << debug.period;
}
Follower::Follower(): m_targetType(TargetType_t::person), m_simmanager_ptr(nullptr), m_stateMachine_st(StateMachine::none), m_runStMachine_st(RunningSubStMachine::unknown),  m_autoNavAlreadyDone(false), m_debugTimePrints(0.0), m_lastValidTargetOnBaseFrame(ReferenceFrameOfTarget_t::mobile_base_body_link)
{
    m_transformData.transformClient = nullptr;
//     m_lastValidTargetOnBaseFrame.first.resize(3, 0.0);
//     m_lastValidTargetOnBaseFrame.second = false;
        m_lostTargetcounter=0;
        m_targetReached=false;
}

Follower::~Follower()
{
    delete m_simmanager_ptr;
}

string Follower::runStMachineState_2_string(RunningSubStMachine st)
{
switch(st)
    {
        case RunningSubStMachine::unknown           :return("RECEIVED_NOTHING")      ;
        case RunningSubStMachine::targetValid       :return("TARGETVALID")      ;
        case RunningSubStMachine::maybeLostTarget   :return("MAYBELOSTTARGET")  ;
        case RunningSubStMachine::lostTarget_lookup :return("LOSTTARGET_LOOKUP");
        case RunningSubStMachine::startAutoNav      :return("STARTAUTONAV")     ;
        case RunningSubStMachine::waitAutoNav       :return("WAITAUTONAV")      ;
        case RunningSubStMachine::autoNavOk         :return("AUTONAVOK")        ;
        case RunningSubStMachine::autoNavError      :return("AUTONAVERROR")     ;
        case RunningSubStMachine::needHelp          :return("NEEDHELP")         ;
        default: return "UNKNOWN RUN-STATE";
    };
}

string Follower::stateMachineState_2_string(StateMachine st)
{

    switch(st)
    {
        case StateMachine::none      :return("NONE");
        case StateMachine::configured:return("CONFIGURED");
        case StateMachine::running   :return("RUNNING");
        case StateMachine::error     :return("ERROR");
        default: return "UNKNOWN STATE";
    };

}


void Follower::setDebug(DebugLevel_t level, bool on)
{
    switch(level)
    {
        case DebugLevel_t::general: m_cfg.debug.enabled=on; break;
        //case DebugLevel_t::targetRetriever:  break; here I haven't the target pointer
        case DebugLevel_t::gazeController: m_gazeCtrl.setDebug(on); break;
        case DebugLevel_t::navigationController: m_navCtrl.setDebug(on); break;
        case DebugLevel_t::ObstacleVerifier: m_obsVer.setDebug(on); break;
        default: return;
    };
}

void Follower::printDebugInfo(Target_t &currenttarget)
{
    if(m_debugTimePrints==0.0)
    {
        m_debugTimePrints=yarp::os::Time::now();
        return;
    }
    if((yarp::os::Time::now()-m_debugTimePrints) <m_cfg.debug.period)
        return;


    string str;
    yCDebug(FOLLOWER) << "****************************************************************";
    yCDebug(FOLLOWER) << "**** Target Type is "<< m_cfg.targetType;
    yCDebug(FOLLOWER) << "****"<< currenttarget.toString();

    yCDebug(FOLLOWER)<< "**** Main State Machine is in " << stateMachineState_2_string(m_stateMachine_st);

    if(m_stateMachine_st == StateMachine::configured)
    {
        yCDebug(FOLLOWER)<< "****    I'm waiting the start command";
    }
    else
    {
        yCDebug(FOLLOWER)<< "**** Running State Machine is in" << runStMachineState_2_string(m_runStMachine_st);
        if(m_obsVer.isRunning())
        {
            if(m_obsVerResult.resultIsValid)
            {
                if(m_obsVerResult.result)
                    yCDebug(FOLLOWER) << "**** ObstacleCheck: I detected an OBSTACLE on my path";
                else
                    yCDebug(FOLLOWER) << "**** ObstacleCheck: No obstacle on my path";
            }
            else
                yCError(FOLLOWER) << "**** ObstacleCheck: an error occured";
        }
        else
            yCDebug(FOLLOWER) << "**** ObstacleCheck: not configured! Attention!!!";

        if(m_runStMachine_st == RunningSubStMachine::lostTarget_lookup)
        {
            yCDebug(FOLLOWER) << "**** GazeController is in " << m_gazeCtrl.stausToStrig();
        }
        yCDebug(FOLLOWER)<< "**** Target is REACHED " << m_targetReached;
    }
    yCDebug(FOLLOWER) << "****************************************************************";

    m_debugTimePrints=yarp::os::Time::now();
}


Result_t Follower::followTarget(Target_t &target)
{
    RunningSubStMachine currSt, newSt;
    SMEvents evt;

    if(m_cfg.debug.enabled)
        printDebugInfo(target);

    if(!isInRunningState())
        return Result_t::notRunning;


    Result_t res=Result_t::error;
    currSt=m_runStMachine_st;
    if(target.isValid)
    {
        evt=SMEvents::validTargetRec;

        goto_targetValid_state();
        res= processValidTarget(target);
        if(res==Result_t::needHelp)
            m_runStMachine_st=RunningSubStMachine::needHelp;
    }
    else
    {
        evt=SMEvents::invalidTargetRec;
        //if the target is not valid.....
        switch(m_runStMachine_st)
        {
            case RunningSubStMachine::targetValid:
            {
                //for the first time I received a not valid target
                m_runStMachine_st=RunningSubStMachine::maybeLostTarget;
                m_lostTargetcounter++;

                res=processTarget_core(m_lastValidTargetOnBaseFrame);
                if(res==Result_t::needHelp)
                    m_runStMachine_st=RunningSubStMachine::needHelp;
            }break;

            case RunningSubStMachine::maybeLostTarget:
            {
                m_lostTargetcounter++;

                if(m_lostTargetcounter>=m_cfg.invalidTargetMax)
                    m_runStMachine_st=RunningSubStMachine::lostTarget_lookup;

                res=processTarget_core(m_lastValidTargetOnBaseFrame);
                if(res==Result_t::needHelp)
                    m_runStMachine_st=RunningSubStMachine::needHelp;
            }break;

            case RunningSubStMachine::lostTarget_lookup :
            {
                res=Result_t::lostTarget;

                GazeCtrlLookupStates gaze_st = m_gazeCtrl.lookup4Target();
                if(GazeCtrlLookupStates::finished == gaze_st)
                {
                    evt=SMEvents::lookupFinished;
                    m_gazeCtrl.resetLookupstateMachine();
                    if(m_cfg.debug.enabled)
                        yCDebug(FOLLOWER) << "I looked around but I cannot find the target!";

                    if((m_autoNavAlreadyDone) || (!m_navCtrl.isConfigured()))
                    {
                        m_runStMachine_st=RunningSubStMachine::needHelp;
                        res=Result_t::failed;
                    }
                    else
                    {
                        m_runStMachine_st=RunningSubStMachine::startAutoNav;
                        res=Result_t::lostTarget;
                    }
                }
                else if(GazeCtrlLookupStates::failed == gaze_st)
                {
                    m_runStMachine_st=RunningSubStMachine::needHelp;
                    res=Result_t::failed;
                }
            }break;

            case RunningSubStMachine::startAutoNav:
            {
                if(false ==  m_lastValidTargetOnBaseFrame.isValid)
                {
                    m_runStMachine_st= RunningSubStMachine::needHelp;
                    if(m_cfg.debug.enabled)
                        yCDebug(FOLLOWER) << "I can't start to auto nav because the last target is not valid";
                    res = Result_t::failed;
                }
                else
                {
                    if(!m_targetReached)
                    {
                        if(m_cfg.debug.enabled)
                            yCDebug(FOLLOWER) << "I lost the target. START AUTONOMOUS NAVIGATION toward the last target="<< m_lastValidTargetOnBaseFrame.point3D[0] << m_lastValidTargetOnBaseFrame.point3D[1];

                        bool resNav = m_navCtrl.startAutonomousNav(m_lastValidTargetOnBaseFrame.point3D[0], m_lastValidTargetOnBaseFrame.point3D[1], 0);

                        //from now the last target is not more valid
                        m_lastValidTargetOnBaseFrame.isValid=false;

                        if(resNav)
                        {
                            m_runStMachine_st= RunningSubStMachine::waitAutoNav;
                            res = Result_t::autoNavigation;
                        }
                        else
                        {
                            m_runStMachine_st= RunningSubStMachine::needHelp;
                            yCError(FOLLOWER) << "Error starting autonomous navigation. I need help";
                            res = Result_t::error;
                        }
                    }
                    else
                    {
                        //m_targetReached=true;
                        //from now the last target is not more valid
                        m_lastValidTargetOnBaseFrame.isValid=false;
                        if(m_cfg.debug.enabled)
                            yCDebug(FOLLOWER) << "Target REACHED!!!!!";
                        res = Result_t::ok;
                    }
                }
            }break;


            case RunningSubStMachine::waitAutoNav:
            {
                NavigationStatusEnum navst = m_navCtrl.getNavigationStatus();

                switch(navst)
                {
                    case navigation_status_preparing_before_move:
                    case navigation_status_moving               :
                    case navigation_status_waiting_obstacle     :
                    case navigation_status_thinking             :
                    {
                        res = Result_t::autoNavigation;
                    }break;

                    case navigation_status_goal_reached         :
                    case navigation_status_idle                 : //the navigator goes in idle state after x milliseconds it had reached the goal
                    {
                        //ok: Now I reached the position where I saw the last valid target, but now I still have a not valid target.
                        //maybe next round I'll be lucky
                        m_runStMachine_st= RunningSubStMachine::needHelp;
                        m_autoNavAlreadyDone=true;
                        if(m_cfg.debug.enabled)
                            yCDebug(FOLLOWER) << "I reached the last valid target with autonomous navigation";
                        evt=SMEvents::invalidTargetRec;
                        res = Result_t::autoNavigation;
                    }break;
                    //TODO: what does the follower do in these cases????
                    case navigation_status_aborted              :
                    case navigation_status_paused               :
                    {
                        m_runStMachine_st= RunningSubStMachine::needHelp;
                        if(m_cfg.debug.enabled)
                            yCDebug(FOLLOWER) << "Autonomous navigation has been aborted or paused. I need help";
                        res = Result_t::needHelp;
                        evt=SMEvents::invalidTargetRec;
                    }break;

                        //navigation module is in error, than I need help
                    case navigation_status_error                :
                    case navigation_status_failing              :
                    {
                        m_runStMachine_st= RunningSubStMachine::needHelp;
                        if(m_cfg.debug.enabled)
                            yCDebug(FOLLOWER) << "Autonomous navigation ended with error. I need help";
                        res = Result_t::error;
                    }break;
                };
            }break;

            case RunningSubStMachine::needHelp:
            {
                /*Nothing to do*/
                //if(m_cfg.debug.enabled)
                //    yCDebug() << "I need help";

                res = Result_t::needHelp;
            }break;
        };//end switch
    } //end else
    newSt=m_runStMachine_st;
    //commented because the error event is not managed by monitor
//    if((res==Result_t::failed) || (res==Result_t::error))
//        evt=SMEvents::error;

    m_transition = std::make_tuple(currSt, newSt, evt);
    return res;
}

bool Follower::configure(yarp::os::ResourceFinder &rf)
{
    m_outputPortJoystick.open("/follower/test-joystick:o");//test


    if(!readConfig(rf, m_cfg))
    {
        yCError(FOLLOWER) << "Error reading configuration file";
        return false;
    }

    if(m_cfg.targetType == "redball")
    {
        m_transformData.targetFrameId = m_transformData.redBallFrameId;
        m_targetType = TargetType_t::redball;
    }
    else if(m_cfg.targetType == "fakehumanmodel")
    {
        m_targetType = TargetType_t::fakeHumanModel;
        //m_transformData ???
    }
    else //person or default
    {
        m_transformData.targetFrameId = m_transformData.personFrameId;
        m_targetType = TargetType_t::person;
    }


    if(!m_outputPort2baseCtr.open("/follower/" + m_cfg.outputPortName + ":o"))
    {
        yCError(FOLLOWER) << "Error opening output port for base control";
        return false;
    }

    GazeCtrlUsedCamera cam = GazeCtrlUsedCamera::depth;
    if(m_targetType==TargetType_t::redball)
        cam = GazeCtrlUsedCamera::left;

    if(!m_gazeCtrl.init( cam, rf))
        return false;

    if(m_cfg.onSimulator)
    {
        m_simmanager_ptr = new SimManager();
        m_simmanager_ptr->init("SIM_CER_ROBOT", "/follower/worldInterface/rpc", m_cfg.debug.enabled);
    }

    if(!initTransformClient())
        return false;

    if(!m_navCtrl.configure(rf))
        return false;

    if(!m_obsVer.configure(rf))
        return false;

    std::lock_guard<std::mutex> lock(m_mutex);
    if(m_cfg.startWithoutCommand)
        m_stateMachine_st=StateMachine::running;
    else
        m_stateMachine_st=StateMachine::configured;
    return true;
}


bool Follower::close()
{

    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st = StateMachine::none;
    m_transformData.driver.close();

    m_outputPort2baseCtr.interrupt();
    m_outputPort2baseCtr.close();

    m_outputPortJoystick.interrupt();
    m_outputPortJoystick.close();

//     m_outputPort2gazeCtr.interrupt();
//     m_outputPort2gazeCtr.close();

    m_gazeCtrl.deinit();

    if(m_simmanager_ptr)
        m_simmanager_ptr->deinit();


    return true;
}


bool Follower::start()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st = StateMachine::running;
    return true;
}


bool Follower::stop()
{
    m_gazeCtrl.lookInFront();
    goto_targetValid_state();
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stateMachine_st = StateMachine::configured;
    m_runStMachine_st = RunningSubStMachine::unknown;
    return true;
}


bool Follower::helpProvided(void)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if(!isInRunningState())
        return false;

    goto_targetValid_state();
    return true;
}



TargetType_t Follower::getTargetType(void)
{
    return m_targetType;
}


StateMachine Follower::getState(void)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stateMachine_st;
}

FollowerTarget::FollowerSMTransition FollowerTarget::Follower::getSmTransion()
{
    return m_transition;
}


//------------------------------------------------
// private function
//------------------------------------------------


// bool Follower::checkTargetIsInThreshold(yarp::sig::Vector &target)
// {
//
//
// }
bool Follower::isInRunningState(void)
{
    return (m_stateMachine_st==StateMachine::running);
}

Result_t Follower::processValidTarget(Target_t &target)
{

    //1. transform the inputTarget in a frame on mobile_base_body_link and left or depth_rbg link
    yarp::sig::Vector targetOnBaseFrame, targetOnCamFrame;


    if(!transformPointInBaseFrame(target, targetOnBaseFrame))
    {
        return Result_t::error;
    }

    //2. save the current target
    m_lastValidTargetOnBaseFrame.point3D=targetOnBaseFrame;
    m_lastValidTargetOnBaseFrame.pixel=target.pixel;
    m_lastValidTargetOnBaseFrame.isValid = true;
    m_lastValidTargetOnBaseFrame.refFrame=ReferenceFrameOfTarget_t::mobile_base_body_link;


    sendtargets4Debug(target.point3D, targetOnBaseFrame);

    Result_t res=processTarget_core(m_lastValidTargetOnBaseFrame);


    if(isOnSimulator() && m_cfg.debug.paintGazeFrame)
    {
        yarp::sig::Vector targetOnHeadFrame;
        if(transformPointInHeadFrame(m_transformData.targetFrameId, target.point3D, targetOnHeadFrame))
        {
            targetOnHeadFrame[2]+=0.20;
            m_simmanager_ptr->PaintGazeFrame(targetOnHeadFrame);
        }
    }

    return res;
}

/*
 Returns:
 - needhelp if there is an obstacle
 - ok
 - nok in case of error in getting laser data
 */
Result_t Follower::processTarget_core(Target_t &targetOnBaseFrame)
{
    //I need to check if the target is not valid in the case I have never seen the target and therefore the last target has never been valid
    if(!targetOnBaseFrame.isValid)
        return Result_t::lostTarget;

    Result_t res=Result_t::ok;
    //check if there is an obstacle on my path

    if(m_obsVer.isRunning())
    {
        m_obsVerResult = m_obsVer.checkObstaclesInPath();
        if(m_obsVerResult.resultIsValid)
        {
            if(m_obsVerResult.result)
            {
                //yCInfo() << "+++++++++++++++++++++++++++++++ OBSTACLE"; //continue
                 res=Result_t::needHelp;
            }
            else
            {
                //yCInfo() << "No obstacle found"; //continue
            }

        }
        else
        {
            yCError(FOLLOWER) << "error reading laser";
            return Result_t::error;
        }


    }

    double lin_vel  = 0.0;
    double ang_vel = 0.0;
    if(res==Result_t::ok) //if no obstacle is found, than calculate linear and angular velocity to get the target
    {
        //3. Calculate linear velocity and angular velocity to send to  base control module

        //x axis is the first element, y is on second. (In order to calculate the distance see mobile base frame)
        double distance =  sqrt(pow(targetOnBaseFrame.point3D[0], 2) + pow(targetOnBaseFrame.point3D[1], 2));

        const double RAD2DEG  = 180.0/M_PI;
        double angle = atan2(targetOnBaseFrame.point3D[1], targetOnBaseFrame.point3D[0]) * RAD2DEG;



        m_targetReached=false;
        bool d_in_thr=false;
        bool a_in_thr=false;
        if(distance > m_cfg.navigation.distanceThreshold)
        {
            lin_vel = m_cfg.navigation.factorDist2Vel *(distance-m_cfg.navigation.distanceThreshold);
        }
        else
        {
//             if(m_cfg.debug.enabled)
//                 yCDebug() <<  "the distance is under threshold!! ";
            d_in_thr = true;
        }


        if(fabs(angle) >m_cfg.navigation.angleThreshold)
        {
            ang_vel = m_cfg.navigation.factorAng2Vel * angle;
        }
        else
        {
//             if(m_cfg.debug.enabled)
//                 yCDebug() <<  "the angle is under threshold!! ";
            a_in_thr=true;
        }

        if(d_in_thr && a_in_thr)
        {
            m_targetReached = true;
//             if(m_cfg.debug.enabled)
//                 yCDebug() << "_______________________________________________Target REACHED!!!";
        }
        else
            m_targetReached = false;

        //if the angle is between in the range of given angleLimitsVelReduction parameter than linear velocity is decreased proportionally to the angle.
        //In this way I give more emphasis to rotation movement on place respect to linear one and avoid  velocity goes to zero in one shot.
        if((fabs(angle) > m_cfg.navigation.angleLimitsVelReduction.min) && (fabs(angle) < m_cfg.navigation.angleLimitsVelReduction.max))
        {
            double v=lin_vel;
            auto a_min=m_cfg.navigation.angleLimitsVelReduction.min;
            auto a_max=m_cfg.navigation.angleLimitsVelReduction.max;
            lin_vel = lin_vel*((a_max-fabs(angle))/(a_max-a_min));
            if(lin_vel<0)
                lin_vel=0;
        }

        //saturate velocities
        if(ang_vel > m_cfg.navigation.velocityLimits.angular)
            ang_vel= m_cfg.navigation.velocityLimits.angular;
        else if(ang_vel < -m_cfg.navigation.velocityLimits.angular)
            ang_vel= -m_cfg.navigation.velocityLimits.angular;

        if(lin_vel> m_cfg.navigation.velocityLimits.linear)
            lin_vel = m_cfg.navigation.velocityLimits.linear;
        else if(lin_vel < -m_cfg.navigation.velocityLimits.linear)
            lin_vel = -m_cfg.navigation.velocityLimits.linear;

//         if(m_cfg.debug.enabled)
//             yCDebug() << "sendCommand2BaseControl linvel=" << lin_vel <<"ang_vel" <<ang_vel << "max_lin="<< m_cfg.navigation.velocityLimits.linear << "max_ang=" << m_cfg.navigation.velocityLimits.angular;
    }

    sendCommand2BaseControl(0.0, lin_vel, ang_vel );

    //4. send commands to gaze control in order to follow the target with the gaze
//     yarp::sig::Vector v_targetOnBaseFrame = {0.0, 0.0, 0.0};
//     v_targetOnBaseFrame[0]= targetOnBaseFrame.first[0];
//     v_targetOnBaseFrame[1]= targetOnBaseFrame.first[1];
//     v_targetOnBaseFrame[2]= targetOnBaseFrame.first[2];
    //m_gazeCtrl.lookAtPoint(targetOnBaseFrame.point3D);
    if(targetOnBaseFrame.hasValidPixel())
        m_gazeCtrl.lookAtPixel(targetOnBaseFrame.pixel[0], targetOnBaseFrame.pixel[1]);
//TODO: check why "look at point" works well on bash and not here.
//     else
//     {
//         yarp::sig::Vector p= targetOnBaseFrame.point3D;
//         p[2]+=1.3;
//         m_gazeCtrl.lookAtPoint(p);// this is less stable on real robot
//     }

    //the following steps lose interest on real robot
    if(!isOnSimulator())
        return res;

    //5. paint in gazebo the targets on final target and on cam(optional)
    yarp::sig::Vector target2Paint=targetOnBaseFrame.point3D;
    target2Paint[2]=0.0; //I don't want z axis
    m_simmanager_ptr->PaintTargetFrame(target2Paint);

    return res;
}






void  Follower::goto_targetValid_state()
{
    m_lostTargetcounter=0;

    m_navCtrl.AbortAutonomousNav();
    if(m_runStMachine_st==RunningSubStMachine::lostTarget_lookup)
    {
        m_gazeCtrl.resetLookupstateMachine();
    }
    m_runStMachine_st=RunningSubStMachine::targetValid;
    m_autoNavAlreadyDone=false;
}


bool Follower::transformPointInBaseFrame(Target_t &validTarget, yarp::sig::Vector &pointOutput)
{
    if(validTarget.refFrame == ReferenceFrameOfTarget_t::mobile_base_body_link)
    {
        pointOutput=validTarget.point3D;
        return true;
    }
    else
    {
        bool res = m_transformData.transformClient->transformPoint( ReferenceFrameOfTarget2String(validTarget.refFrame) , m_transformData.baseFrameId, validTarget.point3D, pointOutput);
        if(res)
        {
            //        yCDebug() << "point (" << pointInput.toString() << ") has been transformed in (" << pointOutput.toString() << ")";
        }
        else
        {
            yCError(FOLLOWER) << "FOLLOWER: error in transformPointInBaseFrame()";
        }
        return res;
    }
}


bool Follower::transformPointInCamFrame(Target_t &validTarget, yarp::sig::Vector &pointOutput)
{
    if(validTarget.refFrame == ReferenceFrameOfTarget_t::depth_rgb)
    {
        pointOutput=validTarget.point3D;
        return true;
    }
    else
    {
        bool res = m_transformData.transformClient->transformPoint( ReferenceFrameOfTarget2String(validTarget.refFrame) , "depth_rgb", validTarget.point3D, pointOutput);
        if(res)
        {
            //        yCDebug() << "point (" << pointInput.toString() << ") has been transformed in (" << pointOutput.toString() << ")";
        }
        else
        {
            yCError(FOLLOWER) << "FOLLOWER: error in transformPointInCamFrame()";
        }
        return res;
    }
}



bool Follower::transformPointInHeadFrame(std::string frame_src, yarp::sig::Vector &pointInput, yarp::sig::Vector &pointOutput)
{
    bool res = m_transformData.transformClient->transformPoint(frame_src, "head_link", pointInput, pointOutput);
    if(res)
    {
        //        yCDebug() << "point (" << pointBallInput.toString() << ") has been transformed in (" << pointBallOutput.toString() << ")";
    }
    else
    {
        yCError(FOLLOWER) << "Error in getting transform point from " << frame_src <<" to head_link";
    }

    return res;
}



bool Follower::readConfig(yarp::os::ResourceFinder &rf, FollowerConfig &cfg)
{
    Bottle config_group = rf.findGroup("FOLLOWER_GENERAL");
    if (config_group.isNull())
    {
        yCWarning(FOLLOWER) << "Missing FOLLOWER_GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("inputPort"))  {cfg.inputPortName = config_group.find("inputPort").asString(); }
        if (config_group.check("outputPort"))  { cfg.outputPortName = config_group.find("outputPort").asString(); }
        if (config_group.check("targetType"))  { cfg.targetType = config_group.find("targetType").asString(); }
        if (config_group.check("startWithoutCommand"))  { cfg.startWithoutCommand= config_group.find("startWithoutCommand").asBool(); }
        if (config_group.check("invalidTargetMax"))  { cfg.invalidTargetMax = config_group.find("invalidTargetMax").asInt32(); }
        if (config_group.check("onSimulator"))  { cfg.onSimulator = config_group.find("onSimulator").asBool(); }
    }


    config_group = rf.findGroup("NAVIGATION");
    if (config_group.isNull())
    {
        yCWarning(FOLLOWER) << "Missing TRAJECTORY group! the module uses default value!";
    }
    else
    {
        if (config_group.check("factorDist2Vel")) { cfg.navigation.factorDist2Vel = config_group.find("factorDist2Vel").asFloat64(); }
        if (config_group.check("factorAng2Vel"))  { cfg.navigation.factorAng2Vel = config_group.find("factorAng2Vel").asFloat64(); }
        if (config_group.check("distanceThreshold"))  { cfg.navigation.distanceThreshold = config_group.find("distanceThreshold").asFloat64(); }
        if (config_group.check("angleThreshold"))  { cfg.navigation.angleThreshold = config_group.find("angleThreshold").asFloat64(); }
        if (config_group.check("angularVelLimit"))  { cfg.navigation.velocityLimits.angular = config_group.find("angularVelLimit").asFloat64(); }
        if (config_group.check("linearVelLimit"))  { cfg.navigation.velocityLimits.linear = config_group.find("linearVelLimit").asFloat64(); }
        if (config_group.check("angleLimitsVelReduction"))
        {
            cfg.navigation.angleLimitsVelReduction.min = config_group.find("angleLimitsVelReduction").asList()->get(0).asFloat64();
            cfg.navigation.angleLimitsVelReduction.max = config_group.find("angleLimitsVelReduction").asList()->get(1).asFloat64();
        }
    }


    config_group = rf.findGroup("DEBUG");
    if (config_group.isNull())
    {
        yCWarning(FOLLOWER) << "Missing DEBUG group! the module uses default value!";
    }
    else
    {
        if (config_group.check("enable")) { cfg.debug.enabled = config_group.find("enable").asBool(); }
        if (config_group.check("paintGazeFrame"))  { cfg.debug.paintGazeFrame = config_group.find("paintGazeFrame").asBool(); }
        if (config_group.check("printPeriod"))  { cfg.debug.period = config_group.find("printPeriod").asFloat64(); }
    }

    cfg.print();
    return true;

}

bool Follower::initTransformClient(void)
{
    // Prepare properties for the FrameTransformClient
    yarp::os::Property propTfClient;
    propTfClient.put("device", TF_CLIENT_DEFAULT_DEVICE);
    propTfClient.put("local", "/transformClient-follower");
    propTfClient.put("remote", "/transformServer");

    // Try to open the driver
    bool ok_open = m_transformData.driver.open(propTfClient);
    if (!ok_open)
    {
        yCError(FOLLOWER) << "Unable to open the FrameTransformClient driver.";
        return false;
    }

    // Try to retrieve the view
    bool ok_view = m_transformData.driver.view(m_transformData.transformClient);
    if (!ok_view || m_transformData.transformClient == 0)
    {
        yCError(FOLLOWER) << "Unable to retrieve the FrameTransformClient view.";
        return false;
    }

    //from now I can use m_transformData.transformClient
    return true;
}


bool Follower::sendCommand2BaseControl(double linearDirection, double linearVelocity, double angularVelocity)
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send velocity commands to the base control
    if (m_outputPort2baseCtr.getOutputCount() > 0)
    {
        Bottle &b = m_outputPort2baseCtr.prepare();
        m_outputPort2baseCtr.setEnvelope(stamp);
        b.clear();
        b.addInt32(2);                    // polar speed commands
        b.addFloat64(linearDirection);    // angle in deg
        b.addFloat64(linearVelocity);    // lin_vel in m/s
        b.addFloat64(angularVelocity);    // ang_vel in deg/s
        b.addFloat64(100);
        m_outputPort2baseCtr.write();
    }

    return true;

}

// bool Follower::sendCommand2GazeControl(double x, double y, double z)
// {
//     if (m_outputPort2gazeCtr.getOutputCount() == 0)
//         return true;
//
//     Property &p = m_outputPort2gazeCtr.prepare();
//     p.clear();
//     p.put("control-frame","gaze");
//     p.put("target-type","cartesian");
//
//     Bottle location = yarp::os::Bottle();
//     Bottle &val = location.addList();
//     val.addFloat64(x);
//     val.addFloat64(y);
//     val.addFloat64(z);
//     p.put("target-location",location.get(0));
//     m_outputPort2gazeCtr.write();
//
//     return true;
//
// }


// bool Follower::sendCommand2GazeControl_lookAtPixel(double u, double v)
// {
//     if (m_outputPort2gazeCtr.getOutputCount() == 0)
//         return true;
//
//     Property &p = m_outputPort2gazeCtr.prepare();
//     p.clear();
//     p.put("control-frame","left");
//     p.put("target-type","image");
//     p.put("image","left");
//
//     Bottle location = yarp::os::Bottle();
//     Bottle &val = location.addList();
//     val.addFloat64(u);
//     val.addFloat64(v);
//     p.put("target-location",location.get(0));
//     m_outputPort2gazeCtr.write();
//
//     return true;
//
// }


// bool Follower::sendCommand2GazeControl_lookAtPoint(const  yarp::sig::Vector &x)
// {
//     if (m_outputPort2gazeCtr.getOutputCount() == 0)
//         return true;
//
//     Property &p = m_outputPort2gazeCtr.prepare();
//     p.clear();
//     if(m_targetType==TargetType_t::person)
//         p.put("control-frame","depth_center");
//     else
//         p.put("control-frame","left");
//
//     p.put("target-type","cartesian");
//
//     Bottle target;
//     target.addList().read(x);
//     p.put("target-location",target.get(0));
//
//     if(m_cfg.debug.enabled)
//         yCDebug() << "Command to gazectrl: " << p.toString();
//
//     m_outputPort2gazeCtr.write();
//
//     return true;
//
// }


///////////////////////////////////////////////////////////////////////////////////////////
//////////////////// TEST FUNCTION ///////////////////////////////////////////////////////

//test function: if you want use it put it in update()
bool Follower::moveRobot(void)
{

    if(m_outputPort2baseCtr.getOutputCount() == 0)
        return true;

    static double absTime = yarp::os::Time::now();
    static double startTime = yarp::os::Time::now();
    static int direction = 1.0;

    double currTime = yarp::os::Time::now();

    if(currTime - absTime >120.0)// after 2 minutes stop robot
    {
        //stop moveRobot
        sendCommand2BaseControl(0.0, 0.0, 0.0);
        //cout << "_";
        return true;

    }
    if(currTime-startTime < 10.0)
    {
        sendCommand2BaseControl(0.0, 0.0, 10.0*direction );
        //cout << ".";
    }
    else
    {
        startTime = yarp::os::Time::now();
        (direction >0)? direction = -1.0 : direction=1.0;
        //cout <<"|";
    }
    //cout.flush();
    return true;
}


void Follower::sendOutputLikeJoystick()
{
    static yarp::os::Stamp stamp;

    //send data to baseControl module
    if (m_outputPortJoystick.getOutputCount() == 0)
    {
        //if I have not connection I don't send anything
        return;
    }
    stamp.update();
    Bottle &b = m_outputPortJoystick.prepare();
    m_outputPortJoystick.setEnvelope(stamp);
    b.clear();
    //like joystick
    b.addInt32(3);//write cartesian speed
    b.addFloat64(100);    // angle in deg
    b.addFloat64(0);    // lin_vel in m/s
    b.addFloat64(0);    // ang_vel in deg/s
    b.addFloat64(100);

    m_outputPortJoystick.write();
}


void Follower::sendtargets4Debug(yarp::sig::Vector &VonCamFrame, yarp::sig::Vector &VonBaseFrame)
{
    static yarp::os::Stamp stamp;

    //send data to baseControl module
    if (m_outputPortJoystick.getOutputCount() == 0)
    {
        //if I have not connection I don't send anything
        return;
    }
    stamp.update();
    Bottle &b = m_outputPortJoystick.prepare();
    m_outputPortJoystick.setEnvelope(stamp);
    b.clear();
    //like joystick
    b.addFloat64(VonCamFrame[0]);
    b.addFloat64(VonCamFrame[1]);
    b.addFloat64(VonCamFrame[2]);

    b.addFloat64(VonBaseFrame[0]);
    b.addFloat64(VonBaseFrame[1]);
    b.addFloat64(VonBaseFrame[2]);


    m_outputPortJoystick.write();
}

bool Follower::getMatrix(yarp::sig::Matrix &transform)
{
    bool res = m_transformData.transformClient->getTransform (m_transformData.targetFrameId, m_transformData.baseFrameId, transform);
    if(res)
    {
        //         yCDebug() << "FOLLOWER: i get the transform matrix:"; // << transform.toString();
        //
        //         std::cout << transform.toString() << std::endl << std::endl;
    }
    else
    {
        yCError(FOLLOWER) << "FOLLOWER: error in getting transform matrix";
    }

    return res;
}
