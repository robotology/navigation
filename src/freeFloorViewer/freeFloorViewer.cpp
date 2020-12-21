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

FreeFloorViewer::FreeFloorViewer() :
    m_period(1.0)
{
}

bool FreeFloorViewer::configure(yarp::os::ResourceFinder &rf)
{
    if(rf.check("period")){m_period = rf.find("period").asFloat32();}

    double threadPeriod = 0.02;
    if(rf.check("thread_period")){threadPeriod = rf.find("thread_period").asFloat32();}
    
    // --------- Temp RGBDSensor config --------- //
    /* This is created to pass to the headOrientator object the correct FOVs and 
     * image sizes
     * */
    yarp::dev::PolyDriver tempPoly;
    yarp::dev::IRGBDSensor* tempIRGBD{nullptr};
    bool okRgbdRf{rf.check("RGBD_SENSOR_CLIENT")};
    if(!okRgbdRf)
    {
        yCError(FREE_FLOOR_VIEWER,"RGBD_SENSOR_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property rgbdProp;
    rgbdProp.fromString(rf.findGroup("RGBD_SENSOR_CLIENT").toString());
    tempPoly.open(rgbdProp);
    if(!tempPoly.isValid())
    {
        yCError(FREE_FLOOR_VIEWER,"Error opening PolyDriver check parameters");
        return false;
    }
    tempPoly.view(tempIRGBD);
    if(!tempIRGBD)
    {
        yCError(FREE_FLOOR_VIEWER,"Error opening iRGBD interface. Device not available");
        return false;
    }
    double horizFOV, verFOV;
    tempIRGBD->getRgbFOV(horizFOV,verFOV);
    int imgHeight = tempIRGBD->getRgbHeight();
    int imgWidth = tempIRGBD->getRgbWidth();

    tempPoly.close(); // Temporary polydriver closed

    bool okHeadRf{rf.check("HEAD_CONTROL_CLIENT")};
    if(!okHeadRf)
    {
        yCError(FREE_FLOOR_VIEWER,"HEAD_CONTROL_CLIENT section missing in ini file");
        return false;
    }
    yarp::os::Property headProp;
    headProp.fromString(rf.findGroup("HEAD_CONTROL_CLIENT").toString());

    headProp.put("img_width",imgWidth);
    headProp.put("img_height",imgHeight);
    headProp.put("hor_fov",horizFOV);
    headProp.put("ver_fov",verFOV);

    m_headOrientator = new HeadOrientator();
    bool okHead{m_headOrientator->configure(headProp)};
    if(!okHead)
    {
        yCError(FREE_FLOOR_VIEWER,"HeadOrientator configuration failed");
        return false;
    }

    std::string headName = "/freeFloorViewer/clicked_head:i";
    if(rf.check("clicked_head_port")){headName = rf.find("clicked_head_port").asString();}
    m_headInputPort.useCallback(*m_headOrientator);
    bool ret = m_headInputPort.open(headName);
    if (!ret)
    {
        yCError(FREE_FLOOR_VIEWER, "Unable to open module ports");
        return false;
    }

    m_innerThread = new FreeFloorThread(threadPeriod,rf);
    bool threadOk = m_innerThread->start();
    if (!threadOk){
        return false;
    }

    std::string posName = "/freeFloorViewer/clicked_pos:i";
    if(rf.check("clicked_pos_port")){posName = rf.find("clicked_pos_port").asString();}
    m_posInputPort.useCallback(*m_innerThread);
    ret = m_posInputPort.open(posName);
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

    m_headOrientator->close();
    m_headInputPort.close();
    m_posInputPort.close();

    return true;
}

double FreeFloorViewer::getPeriod()
{
    return m_period;
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
