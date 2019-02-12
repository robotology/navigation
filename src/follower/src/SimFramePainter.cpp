
/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file SimFramePainter.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include "SimFramePainter.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/SystemClock.h>
using namespace yarp::os;
using namespace FollowerTarget;


bool SimManager::init(std::string robotName, std::string rpcNamePort, bool debugOn)
{
    m_worldInterfacePort_ptr = std::make_shared<yarp::os::RpcClient>();
    if(!m_worldInterfacePort_ptr->open(rpcNamePort)) //"/follower/worldInterface/rpc"
    {
        yError() << "Error opening worldInterface rpc port!!";
        return false;
    }

    gazeFramePainter_ptr = std::make_unique<SimFramePainter>("gazeFrame", robotName+"::head_link" , m_worldInterfacePort_ptr, debugOn);
    targetFramePainter_ptr = std::make_unique<SimFramePainter>("targetFrame", robotName+"::mobile_base_body_link" , m_worldInterfacePort_ptr, debugOn);
    return true;

}

bool SimManager::deinit(void)
{
    gazeFramePainter_ptr->erase();
    targetFramePainter_ptr->erase();
   // SystemClock::delaySystem(0.1); //Do I need it? TODO
    m_worldInterfacePort_ptr->interrupt();
    m_worldInterfacePort_ptr->close();

//     delete gazeFramePainter_ptr;
//     delete targetFramePainter_ptr;

    return true;
}

void SimManager::PaintGazeFrame(const yarp::sig::Vector &point)
{
    gazeFramePainter_ptr->paint(point);
}

void SimManager::PaintTargetFrame(const yarp::sig::Vector &point)
{
    targetFramePainter_ptr->paint(point);
}


void SimFramePainter::paint(const yarp::sig::Vector &point)
{
    if( (!m_isCreated) && (m_worldInterfacePort_ptr->asPort().getOutputCount() >0 ))
    {
        if(m_debugOn)
            yDebug() << "I'm about to create the target frame called " << m_nameOfFrame;
        Bottle cmd, ans;
        cmd.clear();
        ans.clear();

        cmd.addString("makeFrame");
        cmd.addDouble(0.2); //size
        cmd.addDouble(point[0]);
        cmd.addDouble(point[1]); //y
        cmd.addDouble(point[2]); //z
        cmd.addDouble(0); //r
        cmd.addDouble(0); //p
        cmd.addDouble(0); //y
        //orange color on central ball
        cmd.addInt(0); //red
        cmd.addInt(0); //green
        cmd.addInt(0); //blue
        cmd.addString(m_frameIdOfRef); /*head_leopard_left*/ //frame name
        cmd.addString(m_nameOfFrame); //box obj name

        m_worldInterfacePort_ptr->write(cmd, ans);
        if(m_debugOn)
            yDebug() << "follower: makeFrame= " << cmd.toString() << "  Ans=" << ans.toString();

        if(ans.toString() == m_nameOfFrame)
        {
            m_isCreated = true;
            return;
        }
        else
            m_isCreated = false;
    }

    if(!m_isCreated)
    {
        return;
    }

    // Prapare bottle containg command to send in order to get the current position
    Bottle cmdGet, ansGet, cmdSet, ansSet;
    cmdGet.clear();
    ansGet.clear();
    cmdSet.clear();
    ansSet.clear();
    cmdGet.addString("getPose");
    cmdGet.addString(m_nameOfFrame);
    cmdGet.addString(m_frameIdOfRef);
    m_worldInterfacePort_ptr->write(cmdGet, ansGet);
    //read the answer

    //send command for new position
    cmdSet.addString("setPose");
    cmdSet.addString(m_nameOfFrame);
    cmdSet.addDouble(point[0]);
    cmdSet.addDouble(point[1]);
    cmdSet.addDouble(point[2]); // z
    cmdSet.addDouble(ansGet.get(3).asDouble()); // r
    cmdSet.addDouble(ansGet.get(4).asDouble()); // p
    cmdSet.addDouble(ansGet.get(5).asDouble()); // y
    cmdSet.addString(m_frameIdOfRef);
    m_worldInterfacePort_ptr->write(cmdSet, ansSet);

}

void SimFramePainter::erase(void)
{
    if((m_isCreated) && (m_worldInterfacePort_ptr->asPort().getOutputCount() >0 ))
    {
        if(m_debugOn)
            yDebug() << "I'm about to delete the frame called" << m_nameOfFrame;
        Bottle cmd, ans;
        cmd.clear();
        ans.clear();

        cmd.addString("deleteObject");
        cmd.addString(m_nameOfFrame); //box obj name

        m_worldInterfacePort_ptr->write(cmd, ans);
        if(m_debugOn)
            yDebug() << "follower: deleteCmd= " << cmd.toString() << "  Ans=" << ans.toString();
    }
}
