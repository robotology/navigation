/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file GazeController.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "GazeController.h"

#include <math.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Vocab.h>

using namespace yarp::os;
using namespace FollowerTarget;



bool GazeController::init(GazeCtrlUsedCamera cam, yarp::os::ResourceFinder &rf, bool debugOn)
{
    Bottle gaze_group = rf.findGroup("GAZE");
    if (gaze_group.isNull())
    {
        yWarning() << "Missing GAZE group! the module uses default value!";
    }
    else
    {
        if(gaze_group.check("pixel_x_range"))
        {
            xpixelRange.first=gaze_group.find("pixel_x_range").asList()->get(0).asInt();
            xpixelRange.second=gaze_group.find("pixel_x_range").asList()->get(1).asInt();
        }

        if(gaze_group.check("pixel_y_range"))
        {
            ypixelRange.first=gaze_group.find("pixel_y_range").asList()->get(0).asInt();
            ypixelRange.second=gaze_group.find("pixel_y_range").asList()->get(1).asInt();
        }

        if(gaze_group.check("trajTimeInLookingup"))
        {
            m_trajectoryTime=gaze_group.find("trajTimeInLookingup").asDouble();
        }
        else
            m_trajectoryTime = 10;//sec

            //TODO leggi T default
    }

    yDebug() << "GAZE=" << xpixelRange.first << xpixelRange.second << ypixelRange.first<< ypixelRange.second;
    yDebug() << "GAZE::TRAJECTORYtIME=" << m_trajectoryTime << "default=" <<m_trajectoryTimeDefault;



    if(!m_outputPort2gazeCtr.open("/follower/gazetargets:o"))
    {
        yError() << "Error opening output port for gaze control";
        return false;
    }
    //TODO: read frpm cam the size of image
    m_pLeft.u = 50;
    m_pLeft.v = 110;

    m_pCenter.u = 160;
    m_pCenter.v = 110;

    m_pRight.u = 270;
    m_pRight.v =110;

    if(GazeCtrlUsedCamera::left ==cam)
        m_camera_str_command = "left";
    else
        m_camera_str_command = "depth_rgb";

    m_debugOn=debugOn;

    m_rpcPort2gazeCtr.open("/follower/gazeController/rpc");

    return true;
}

bool GazeController::deinit(void)
{
    m_outputPort2gazeCtr.interrupt();
    m_outputPort2gazeCtr.close();

    m_rpcPort2gazeCtr.interrupt();
    m_rpcPort2gazeCtr.close();
    return true;
}

GazeCtrlLookupStates GazeController::lookup4Target(void)
{
    switch(m_lookupState)
    {
        case GazeCtrlLookupStates::none:
        {
            //calculate the nearest side TODO currently left always
            //lookAtPixel(m_pLeft.u, m_pLeft.v);

            if(m_debugOn)
                yDebug() << "GazeCtrl in NONE state: move gaze to angle (35.0 10.0). TarjectoryTime=" <<m_trajectoryTime ;

            setTrajectoryTime(m_trajectoryTime);
            lookAtAngle(35.0, 10.0);
            m_lookupState = GazeCtrlLookupStates::nearest;
            m_stateMachineTimeOut.starttime = yarp::os::Time::now();
            m_stateMachineTimeOut.isstarted=true;
            m_stateMachineTimeOut.duration=m_trajectoryTime + m_trajectoryTime/100*20;;
        }break;

        case GazeCtrlLookupStates::nearest:
        {
            if(checkMotionDone())
            {
                lookAtAngle(-35.0, 10.0);
                m_lookupState = GazeCtrlLookupStates::otherside;
                m_stateMachineTimeOut.starttime = yarp::os::Time::now();
                if(m_debugOn)
                    yDebug() << "GazeCtrl in NEAREST state: motion completed. Move gaze to angle (-35.0 10.0) ";
            }
            else
            {
                if(yarp::os::Time::now()-m_stateMachineTimeOut.starttime > m_stateMachineTimeOut.duration)
                {
                    m_lookupState=GazeCtrlLookupStates::failed;
                    yError() << "GazeCtrl in NEAREST state: error reaching angle (35.0 10.0)";
                }
                //else do nothing...wait until gaze is in position or there is a timeout.
            }

        }break;


        case GazeCtrlLookupStates::otherside:
        {

            if(checkMotionDone())
            {
                lookAtAngle(0, 10);
                m_lookupState = GazeCtrlLookupStates::infront;
                m_stateMachineTimeOut.starttime = yarp::os::Time::now();
                if(m_debugOn)
                    yDebug() << "GazeCtrl in OTHERSIDE state: motion completed. Move gaze to angle (0.0 10.0) ";
            }
            else
            {
                if(yarp::os::Time::now()-m_stateMachineTimeOut.starttime > m_stateMachineTimeOut.duration)
                {
                    m_lookupState=GazeCtrlLookupStates::failed;
                    yError() << "GazeCtrl in OTHERSIDE state: error reaching angle (35.0 10.0)";
                }
                //else do nothing...wait until gaze is in position or there is a timeout.
            }
         }break;

        case GazeCtrlLookupStates::infront:
        {

            if(checkMotionDone())
            {
                m_lookupState = GazeCtrlLookupStates::finished;
                if(m_debugOn)
                    yDebug() << "GazeCtrl in INFRONT state: motion completed. SUCCESS!";
            }
            else
            {
                if(yarp::os::Time::now()-m_stateMachineTimeOut.starttime > m_stateMachineTimeOut.duration)
                {
                    m_lookupState=GazeCtrlLookupStates::failed;
                    yError() << "GazeCtrl in INFRONT state: error reaching angle (35.0 10.0)";
                }
                //else do nothing...wait until gaze is in position or there is a timeout.
            }
        }break;

        case GazeCtrlLookupStates::finished:
        {
            if(m_debugOn)
                yDebug() << "GazeCtrl in FINISHED state. ";
        }break;
    };
    return m_lookupState;
}


void GazeController::resetLookupstateMachine(void)
{

    if(m_lookupState != GazeCtrlLookupStates::none)
    {
        setTrajectoryTime(m_trajectoryTimeDefault);
        stopLookup4Target();
    }
    m_lookupState = GazeCtrlLookupStates::none;
}

bool GazeController::lookInFront(void)
{
    return lookAtPixel(m_pCenter.u, m_pCenter.v);
}

bool GazeController::lookAtPixel(double u, double v)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    if(isnan(u) || isnan(v))
        return true;

    bool movegaze=false;
    if( (u<xpixelRange.first) || (u>xpixelRange.second))
        movegaze=true;
    if( (v<ypixelRange.first) || (v>ypixelRange.second) )
        movegaze=true;

//     if(m_debugOn)
//         yDebug() << "LookAtPixel(" <<u<<v << ") movegaze=" <<movegaze;

    if(!movegaze)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();
    p.put("control-frame",m_camera_str_command);
    p.put("target-type","image");
    p.put("image",m_camera_str_command);

    Bottle location = yarp::os::Bottle();
    Bottle &val = location.addList();
    val.addDouble(u);
    val.addDouble(v);
    p.put("target-location",location.get(0));
    m_outputPort2gazeCtr.write();

    return true;
}

bool GazeController::lookAtPoint(const  yarp::sig::Vector &x)
{
    static int count=0;
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    count++;
    if(count<20)
        return true;

    count=0;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();

    //p.put("control-frame", m_camera_str_command);
    p.put("target-type","cartesian");

    Bottle target;
    target.addList().read(x);
    p.put("target-location",target.get(0));

//     if(m_debugOn)
//         yDebug() << "Command to gazectrl lookAtPoint: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;
}


bool GazeController::lookAtAngle(double a, double b)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();

    p.put("target-type","angular");

    Bottle target;
    Bottle &val = target.addList();
    val.addDouble(a);
    val.addDouble(b);
    p.put("target-location",target.get(0));

//    if(m_debugOn)
//        yDebug() << "Command to gazectrl lookAtAngle: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;
}


bool GazeController::stopLookup4Target(void)
{
    if (m_rpcPort2gazeCtr.asPort().getOutputCount() == 0)
        return true;

    Bottle cmd, ans;
    cmd.clear();
    ans.clear();

    cmd.addString("stop");

    m_rpcPort2gazeCtr.write(cmd, ans);

   if(m_debugOn)
       yError() << "GazeController::stopLookup4Target rpc_cmd=" << cmd.toString() << "Ans=" << ans.toString();

    if(ans.toString() == "ack")
        return true;
    else
        return false;
}

bool GazeController::setTrajectoryTime(double T)
{
    if (m_rpcPort2gazeCtr.asPort().getOutputCount() == 0)
        return true;

    Bottle cmd, ans;
    cmd.clear();
    ans.clear();

    cmd.addString("set");
    cmd.addString("T");
    cmd.addDouble(T);

    m_rpcPort2gazeCtr.write(cmd, ans);

//    if(m_debugOn)
//        yDebug() << "GazeController: rpc_cmd=" << cmd.toString() << "Ans=" << ans.toString();

    if(ans.toString() == "ack")
        return true;
    else
        return false;

}

bool GazeController::checkMotionDone(void)
{
    if(m_rpcPort2gazeCtr.asPort().getOutputCount() == 0)
        return true;

    Bottle cmd, ans;
    cmd.clear();
    ans.clear();

    cmd.addString("get");
    cmd.addString("done");

    m_rpcPort2gazeCtr.write(cmd, ans);

//    if(m_debugOn)
//        yDebug() << "GazeController: rpc_cmd=" << cmd.toString() << "Ans=" << ans.toString();

    if(ans.get(0).asVocab() == yarp::os::Vocab::encode("ack"))
    {
        if(ans.get(1).asInt() == 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
        return false;


}




bool GazeController::lookAtPointRPC(const  yarp::sig::Vector &x)
{
    if(m_rpcPort2gazeCtr.asPort().getOutputCount() == 0)
        return true;

static int count=0;

count++;
if(count <10)
    return true;

count =0;

    Bottle cmd, ans;
    cmd.clear();
    ans.clear();

    cmd.addString("look");
    Property &p=cmd.addDict();



    p.put("target-type","cartesian");

    Bottle target;
    target.addList().read(x);
    p.put("target-location",target.get(0));


//     if(m_debugOn)
//         yDebug() << "Command to gazectrl lookAtPointRPC: " << cmd.toString();



    m_rpcPort2gazeCtr.write(cmd, ans);

    if(ans.get(0).asVocab() == yarp::os::Vocab::encode("ack"))
    {
        return true;
    }
    else
        return false;

}


