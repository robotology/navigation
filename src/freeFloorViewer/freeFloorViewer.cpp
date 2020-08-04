/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
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
