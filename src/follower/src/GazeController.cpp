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

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/SystemClock.h>

using namespace yarp::os;
using namespace FollowerTarget;



bool GazeController::init(GazeCtrlUsedCamera cam, yarp::os::ResourceFinder &rf, bool debugOn)
{
    Bottle gaze_group = rf.findGroup("GAZE");
    if (gaze_group.isNull())
    {
        yWarning() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (gaze_group.check("pixel_x_range"))
        {
            xpixelRange.first=gaze_group.find("pixel_x_range").asList()->get(0).asInt();
            xpixelRange.second=gaze_group.find("pixel_x_range").asList()->get(1).asInt();
        }

        if (gaze_group.check("pixel_y_range"))
        {
            ypixelRange.first=gaze_group.find("pixel_y_range").asList()->get(0).asInt();
            ypixelRange.second=gaze_group.find("pixel_y_range").asList()->get(1).asInt();
        }
    }

    yError() << "GAZE=" << xpixelRange.first << xpixelRange.second << ypixelRange.first<< ypixelRange.second;




    if(!m_outputPort2gazeCtr.open("/follower/gazetargets:o"))
    {
        yError() << "Error opening output port for gaze control";
        return false;
    }
    //TODO: read frpm cam the size of image
    m_pLeft.u = 630;
    m_pLeft.v = 240;

    m_pCenter.u = 320;
    m_pCenter.v = 240;

    m_pRight.u = 10;
    m_pRight.v =240;

    if(GazeCtrlUsedCamera::left ==cam)
        m_camera_str_command = "left";
    else
        m_camera_str_command = "depth_rgb";

    m_debugOn=debugOn;

    return true;
}
bool GazeController::deinit(void)
{
    m_outputPort2gazeCtr.interrupt();
    m_outputPort2gazeCtr.close();
    return true;
}

GazeCtrlLookupStates GazeController::lookupTarget(void)
{
    switch(m_lookupState)
    {
        case GazeCtrlLookupStates::none:
        {
            //calculate the nearest side TODO currently left always
            //lookAtPixel(m_pLeft.u, m_pLeft.v);
            lookAtAngle(41.8090248586633, 3.42524672670162);
            m_lookupState = GazeCtrlLookupStates::nearest;
            yDebug() << "GazeCtrl in NONE state. ";
            SystemClock::delaySystem(2.5);
        }break;

        case GazeCtrlLookupStates::nearest:
        {
            //lookAtPixel(m_pRight.u, m_pRight.v);
            lookAtAngle(-41.796462338564, 3.42371127637749);
            m_lookupState = GazeCtrlLookupStates::otherside;
            yDebug() << "GazeCtrl in NEAREST state. ";
            SystemClock::delaySystem(2.5);
        }break;

        case GazeCtrlLookupStates::otherside:
        {
            //lookInFront();
            lookAtAngle(0, 0);
            m_lookupState = GazeCtrlLookupStates::finished;
            yDebug() << "GazeCtrl in OTHERSIDE state. look in front ";
            SystemClock::delaySystem(2.5);
        }break;

        case GazeCtrlLookupStates::finished:
        {
            //TODO Do I need to reset the state machine?
            yDebug() << "GazeCtrl in FINISHED state. ";

        }break;
    };
    return m_lookupState;
}
void GazeController::resetLookupstateMachine(void)
{
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

    bool isOutsideRange=false;
    if( (u<xpixelRange.first) || (u>xpixelRange.second))
        isOutsideRange=true;
    if( (v<ypixelRange.first) || (v>ypixelRange.second) )
        isOutsideRange=true;

    yError() << "PIXEL " <<u<<v << "outside=" <<isOutsideRange;
    if(!isOutsideRange)
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
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();

//     if(m_targetType==FollowerTargetType::person)
//         p.put("control-frame","depth_center");
//     else
//         p.put("control-frame","left");

    p.put("control-frame", m_camera_str_command);
    p.put("target-type","cartesian");

    Bottle target;
    target.addList().read(x);
    p.put("target-location",target.get(0));

//     if(m_debugOn)
//         yDebug() << "Command to gazectrl: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;
}


bool GazeController::lookAtAngle(double a, double b)
{
    if (m_outputPort2gazeCtr.getOutputCount() == 0)
        return true;

    Property &p = m_outputPort2gazeCtr.prepare();
    p.clear();

    //     if(m_targetType==FollowerTargetType::person)
    //         p.put("control-frame","depth_center");
    //     else
    //         p.put("control-frame","left");

    //p.put("control-frame", m_camera_str_command);
    p.put("target-type","angular");

    Bottle target;
    Bottle &val = target.addList();
    val.addDouble(a);
    val.addDouble(b);
    p.put("target-location",target.get(0));

    if(m_debugOn)
        yDebug() << "Command to gazectrl: " << p.toString();

    m_outputPort2gazeCtr.write();

    return true;
}
