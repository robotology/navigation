/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
    yarp::os::Property rgbdProp;
    // Prepare default prop object
    rgbdProp.put("device", RGBDClient);
    rgbdProp.put("localImagePort", RGBDLocalImagePort);
    rgbdProp.put("localDepthPort", RGBDLocalDepthPort);
    rgbdProp.put("localRpcPort", RGBDLocalRpcPort);
    rgbdProp.put("remoteImagePort", RGBDRemoteImagePort);
    rgbdProp.put("remoteDepthPort", RGBDRemoteDepthPort);
    rgbdProp.put("remoteRpcPort", RGBDRemoteRpcPort);
    rgbdProp.put("ImageCarrier", RGBDImageCarrier);
    rgbdProp.put("DepthCarrier", RGBDDepthCarrier);
    bool okRgbdRf = rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCWarning(FREE_FLOOR_VIEWER,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
    }
    else
    {
        yarp::os::Searchable& rgbd_config = rf.findGroup("RGBD_SENSOR_CLIENT");
        if(rgbd_config.check("device")) {rgbdProp.put("device", rgbd_config.find("device").asString());}
        if(rgbd_config.check("localImagePort")) {rgbdProp.put("localImagePort", rgbd_config.find("localImagePort").asString());}
        if(rgbd_config.check("localDepthPort")) {rgbdProp.put("localDepthPort", rgbd_config.find("localDepthPort").asString());}
        if(rgbd_config.check("localRpcPort")) {rgbdProp.put("localRpcPort", rgbd_config.find("localRpcPort").asString());}
        if(rgbd_config.check("remoteImagePort")) {rgbdProp.put("remoteImagePort", rgbd_config.find("remoteImagePort").asString());}
        if(rgbd_config.check("remoteDepthPort")) {rgbdProp.put("remoteDepthPort", rgbd_config.find("remoteDepthPort").asString());}
        if(rgbd_config.check("remoteRpcPort")) {rgbdProp.put("remoteRpcPort", rgbd_config.find("remoteRpcPort").asString());}
        if(rgbd_config.check("ImageCarrier")) {rgbdProp.put("ImageCarrier", rgbd_config.find("ImageCarrier").asString());}
        if(rgbd_config.check("DepthCarrier")) {rgbdProp.put("DepthCarrier", rgbd_config.find("DepthCarrier").asString());}
    }
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

    yarp::os::Property headProp;
    // Prepare default prop object
    headProp.put("device","remote_controlboard");
    headProp.put("local","/freeFloorViewer/head");
    headProp.put("remote","/SIM_CER_ROBOT/head");

    bool okHeadRf{rf.check("HEAD_CONTROL_CLIENT")};
    if(!okHeadRf)
    {
        yCError(FREE_FLOOR_VIEWER,"HEAD_CONTROL_CLIENT section missing in ini file. Using default values.");
    }
    else
    {
        yarp::os::Searchable& head_config = rf.findGroup("HEAD_CONTROL_CLIENT");
        if(head_config.check("device")) {headProp.put("device", head_config.find("device").asString());}
        if(head_config.check("local")) {headProp.put("local", head_config.find("local").asString());}
        if(head_config.check("remote")) {headProp.put("remote", head_config.find("remote").asString());}
    }

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
    if (!ret)
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
