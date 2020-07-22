/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \section navigationGUI
 * To be written.
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>

#include "freeFloorViewer.h"

YARP_LOG_COMPONENT(FREE_FLOOR_VIEWER, "navigation.freeFloorViewer")

FreeFloorViewer::FreeFloorViewer()
{
}

bool FreeFloorViewer::configure(yarp::os::ResourceFinder &rf)
{
    double threadPeriod = 0.02;
    if(rf.check("period")){threadPeriod = rf.find("period").asFloat32();}

    std::string rpcName = "/freeFloorViewer/rpc";
    if(rf.check("rpc_port")){rpcName = rf.find("rpc_port").asString();}
    bool ret = m_rpcPort.open(rpcName);
    if (ret == false)
    {
        yCError(FREE_FLOOR_VIEWER, "Unable to open module ports");
        return false;
    }
    attach(m_rpcPort);

    m_innerThread = new FreeFloorThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }

    return true;
}

bool FreeFloorViewer::interruptModule()
{
    m_rpcPort.interrupt();

    return true;
}

bool FreeFloorViewer::close()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();

    m_innerThread->stop();
    delete m_innerThread;
    m_innerThread =NULL;

    return true;
}

double FreeFloorViewer::getPeriod()
{
    return 3.0;
}

bool FreeFloorViewer::updateModule()
{
    if (isStopping())
    {
        if (m_innerThread) m_innerThread->stop();
        return false;
    }

    return true;
}

bool FreeFloorViewer::respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply)
{
    std::lock_guard<std::mutex> lock(m_innerThread->m_floorMutex);
    if (m_rpcPort.isOpen() == false) return false;

    reply.clear();

    if (command.get(0).isString())
    {
        if (command.get(0).asString()=="quit")
        {
            return false;
        }

        else if (command.get(0).asString()=="help")
        {
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("quit");
            reply.addString("draw_locations <0/1>");
        }
        else if (command.get(0).isString())
        {
            yCDebug(FREE_FLOOR_VIEWER, "Not yet implemented.");
        }
    }
    else
    {
        yCError(FREE_FLOOR_VIEWER,"Invalid command type");
        reply.addVocab(VOCAB_ERR);
    }
    return true;
}
