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


#include "freeFloorViewer.h"

YARP_LOG_COMPONENT(FREE_FLOOR_VIEWER, "navigation.freeFloorViewer")

FreeFloorViewer::FreeFloorViewer()
{
}

bool FreeFloorViewer::configure(yarp::os::ResourceFinder &rf)
{
    double threadPeriod = 0.02;
    if(rf.check("period")){threadPeriod = rf.find("period").asFloat32();}

    m_innerThread = new FreeFloorThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }

    std::string posName = "/freeFloorViewer/clicked_pos:i";
    if(rf.check("clicked_pos_port")){posName = rf.find("clicked_pos_port").asString();}
    m_posInputPort.useCallback(*m_innerThread);
    bool ret = m_posInputPort.open(posName);
    if (ret == false)
    {
        yCError(FREE_FLOOR_VIEWER, "Unable to open module ports");
        return false;
    }

    return true;
}

bool FreeFloorViewer::close()
{
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
