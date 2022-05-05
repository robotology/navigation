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


#define _USE_MATH_DEFINES

#include "freeFloorThread.h"
#include <chrono>
#include <cmath>

YARP_LOG_COMPONENT(FREE_FLOOR_THREAD, "navigation.freeFloorViewer.freeFloorThread")
bool print{true};


FreeFloorThread::FreeFloorThread(double _period, yarp::os::ResourceFinder &rf):
    PeriodicThread(_period),
    TypedReaderCallback(),
    m_rf(rf)
{
    m_depth_width = 0;
    m_depth_height = 0;
    m_pc_stepx = 1;
    m_pc_stepy = 1;
    m_col_granularity = 10;
    m_floor_height = 0.1;
    m_ceiling_height = 3.0;
    m_ground_frame_id = "/ground_frame";
    m_camera_frame_id = "/depth_camera_frame";
    m_extern_ref_frame_id = "";
    m_imgOutPortName = "/freeFloorViewer/floorEnhanced:o";
    m_targetOutPortName = "/free_floor_viewer/target:o";
    m_baseCmdOutPortName = "/freeFloorViewer/baseVelCmd:o";
    m_baseCmdInPortName = "/freeFloorViewer/baseCtrlCmd:i";
}

bool FreeFloorThread::threadInit()
{
#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
    if(m_rf.check("target_pos_port")) {m_targetOutPortName = m_rf.find("target_pos_port").asString();}
    if(m_rf.check("img_out_port")) {m_imgOutPortName = m_rf.find("img_out_port").asString();}
    if(m_rf.check("self_reliant")) {m_self_reliant = m_rf.find("self_reliant").asInt()==1;}

    // --------- BaseControl config --------- //
    bool okBaseCtrl = m_rf.check("BASE_CONTROL");
    if(okBaseCtrl){
        if(m_rf.check("base_ctrl_port")) {m_baseCmdOutPortName = m_rf.find("base_ctrl_port").asString();}
        if(m_rf.check("base_ctrl_state_port")) {m_baseCmdInPortName = m_rf.find("base_ctrl_state_port").asString();}
        if(m_rf.check("max_angular_vel")) {m_maxVelTheta = m_rf.find("max_angular_vel").asFloat64();}
        if(m_rf.check("max_linear_vel")) {m_maxVelX = m_rf.find("max_linear_vel").asFloat64();}
        m_outputBaseData.vel_x = 0.0;
        m_outputBaseData.vel_y = 0.0;
        m_outputBaseData.vel_theta = 0.0;
    }

    // --------- Z related props -------- //
    bool okZClipRf = m_rf.check("Z_CLIPPING_PLANES");
    if(okZClipRf)
    {
        yarp::os::Searchable& pointcloud_clip_config = m_rf.findGroup("Z_CLIPPING_PLANES");
        if (pointcloud_clip_config.check("floor_height"))   {m_floor_height = pointcloud_clip_config.find("floor_height").asFloat64();}
        if (pointcloud_clip_config.check("ceiling_height"))   {m_ceiling_height = pointcloud_clip_config.find("ceiling_height").asFloat64();}
        if (pointcloud_clip_config.check("ground_frame_id")) {m_ground_frame_id = pointcloud_clip_config.find("ground_frame_id").asString();}
        if (pointcloud_clip_config.check("camera_frame_id")) {m_camera_frame_id = pointcloud_clip_config.find("camera_frame_id").asString();}
        if (pointcloud_clip_config.check("column_granularity")) {m_col_granularity = pointcloud_clip_config.find("column_granularity").asInt32();}
        if(!m_self_reliant)
        {
            if (pointcloud_clip_config.check("extern_ref_frame_id"))
            {
                m_extern_ref_frame_id = pointcloud_clip_config.find("extern_ref_frame_id").asString();
            }
            else
            {
                yCError(FREE_FLOOR_THREAD, "No exter_ref_frame_id parameter found. Exiting");
                return false;
            }
        }
    }

    // --------- Point cloud quality -------- //
    bool okPCQuality = m_rf.check("POINTCLOUD_QUALITY");
    if(okPCQuality)
    {
        yarp::os::Searchable& pointcloud_qual_config = m_rf.findGroup("POINTCLOUD_QUALITY");
        if (pointcloud_qual_config.check("x_step"))   {m_pc_stepx = pointcloud_qual_config.find("x_step").asInt32();}
        if (pointcloud_qual_config.check("y_step"))   {m_pc_stepy = pointcloud_qual_config.find("y_step").asInt32();}
    }

    // --------- RGBDSensor config --------- //
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
    bool okRgbdRf = m_rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCWarning(FREE_FLOOR_THREAD,"RGBD_SENSOR_CLIENT section missing in ini file. Using default values");
    }
    else
    {
        yarp::os::Searchable& rgbd_config = m_rf.findGroup("RGBD_SENSOR_CLIENT");
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

    m_rgbdPoly.open(rgbdProp);
    if(!m_rgbdPoly.isValid())
    {
        yCError(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_rgbdPoly.view(m_iRgbd);
    if(!m_iRgbd)
    {
        yCError(FREE_FLOOR_THREAD,"Error opening iRGBD interface. Device not available");
        return false;
    }
    //Verify if this is needed
    yarp::os::Time::delay(0.1);

    // --------- TransformClient config FAKE REMOVE WHEN READY--------- //
    yarp::os::Property tcProp;
    // Prepare default prop object
    tcProp.put("device", "frameTransformClient");
    tcProp.put("ft_client_prefix", "/freeFloorViewer");
    tcProp.put("local_rpc", "/freeFloorViewer/ftClient.rpc");
    tcProp.put("filexml_option","ftc_yarp_only.xml");
    bool okTransformRf = m_rf.check("TRANSFORM_CLIENT");
    if(!okTransformRf)
    {
        yCWarning(FREE_FLOOR_THREAD,"TRANSFORM_CLIENT section missing in ini file Using default values");
    }
    else {
        yarp::os::Searchable &tf_config = m_rf.findGroup("TRANSFORM_CLIENT");
        if (tf_config.check("ft_client_prefix")) {
            tcProp.put("ft_client_prefix", tf_config.find("ft_client_prefix").asString());
        }
        if (tf_config.check("ft_server_prefix")) {
            tcProp.put("ft_server_prefix", tf_config.find("ft_server_prefix").asString());
        }
        if(tf_config.check("filexml_option")) {tcProp.put("filexml_option", tf_config.find("filexml_option").asString());}
    }
    m_tcPoly.open(tcProp);
    if(!m_tcPoly.isValid())
    {
        yCError(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_tcPoly.view(m_iTc);
    if(!m_iTc)
    {
        yCError(FREE_FLOOR_THREAD,"Error opening iFrameTransform interface. Device not available");
        return false;
    }

    // --------- Navigation2DClient config --------- //
    yarp::os::Property nav2DProp;
    // Prepare default prop object
    nav2DProp.put("device",         NAVIGATION_CLIENT_DEVICE_DEFAULT);
    nav2DProp.put("local",          NAVLOCAL);
    nav2DProp.put("navigation_server", NAVIGATION_REMOTE_PORT_DEFAULT);
    nav2DProp.put("map_locations_server", MAP_REMOTE_PORT_DEFAULT);
    nav2DProp.put("localization_server", LOCALIZATION_REMOTE_PORT_DEFAULT);
    bool okNavigation2DRf = m_rf.check("NAVIGATION_CLIENT");
    if(!okNavigation2DRf)
    {
        yCWarning(FREE_FLOOR_THREAD,"NAVIGATION_CLIENT section missing in ini file. Using the default values");
    }
    else
    {
        yarp::os::Searchable& nav_config = m_rf.findGroup("NAVIGATION_CLIENT");
        if(nav_config.check("local")) {nav2DProp.put("local", nav_config.find("local").asString());}
        if(nav_config.check("navigation_server")) {nav2DProp.put("navigation_server", nav_config.find("navigation_server").asString());}
        if(nav_config.check("map_locations_server")) {nav2DProp.put("map_locations_server", nav_config.find("map_locations_server").asString());}
        if(nav_config.check("localization_server")) {nav2DProp.put("localization_server", nav_config.find("localization_server").asString());}
    }
    m_nav2DPoly.open(nav2DProp);
    if(!m_nav2DPoly.isValid())
    {
        yCWarning(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters. Using the default values");
    }
    m_nav2DPoly.view(m_iNav2D);
    if(!m_iNav2D){
        yCError(FREE_FLOOR_THREAD,"Error opening iFrameTransform interface. Device not available");
        return false;
    }

    //Verify if this is needed
    yarp::os::Time::delay(0.1);

    //get parameters data from the camera
    m_depth_width = m_iRgbd->getRgbWidth();
    m_depth_height = m_iRgbd->getRgbHeight();
    bool propintr  = m_iRgbd->getDepthIntrinsicParam(m_propIntrinsics);
    if(!propintr){
        return false;
    }
    yCInfo(FREE_FLOOR_THREAD) << "Depth Intrinsics:" << m_propIntrinsics.toString();
    m_intrinsics.fromProperty(m_propIntrinsics);

    if(!m_imgOutPort.open(m_imgOutPortName)){
        yCError(FREE_FLOOR_THREAD) << "Cannot open imgOut port with name" << m_imgOutPortName;
        return false;
    }
    if(!m_targetOutPort.open(m_targetOutPortName)){
        yCError(FREE_FLOOR_THREAD) << "Cannot open targetOut port with name" << m_targetOutPortName;
        return false;
    }
    if(!m_baseCmdOutPort.open(m_baseCmdOutPortName)){
        yCError(FREE_FLOOR_THREAD) << "Cannot open baseCmdOut port with name" << m_baseCmdOutPortName;
        return false;
    }
    if(!m_baseCmdInPort.open(m_baseCmdInPortName)){
        yCError(FREE_FLOOR_THREAD) << "Cannot open baseCmdIn port with name" << m_baseCmdInPortName;
        return false;
    }

#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "... done!\n");
#endif

    return true;
}

void FreeFloorThread::run()
{
    //std::lock_guard<std::mutex> lock(m_floorMutex);
    bool depth_ok = m_iRgbd->getDepthImage(m_depth_image);
    if (!depth_ok)
    {
        yCDebug(FREE_FLOOR_THREAD, "getDepthImage failed");
        return;
    }
    if (m_depth_image.getRawImage()==nullptr)
    {
        yCDebug(FREE_FLOOR_THREAD, "invalid image received");
        return;
    }

    bool rgb_ok = m_iRgbd->getRgbImage(m_rgbImage);
    if (!rgb_ok)
    {
        yCDebug(FREE_FLOOR_THREAD, "getRgbImage failed");
        return;
    }
    if (m_rgbImage.getRawImage()==nullptr)
    {
        yCDebug(FREE_FLOOR_THREAD, "invalid image received");
        return;
    }

    if (m_depth_image.width()!=m_depth_width ||
        m_depth_image.height()!=m_depth_height)
    {
        yCDebug(FREE_FLOOR_THREAD,"invalid image size: (%d %d) vs (%d %d)",m_depth_image.width(),m_depth_image.height(),m_depth_width,m_depth_height);
        return;
    }

    //we compute the transformation matrix from the camera to the laser reference frame

    bool frame_exists = m_iTc->getTransform(m_camera_frame_id,m_ground_frame_id, m_transform_mtrx);
    if (!frame_exists)
    {
        yCWarning(FREE_FLOOR_THREAD, "Unable to found m matrix");
    }
    if(!m_self_reliant)
    {
        frame_exists = m_iTc->getTransform(m_camera_frame_id, m_extern_ref_frame_id, m_transform_mtrx_extern);
        if (!frame_exists) {
            yCWarning(FREE_FLOOR_THREAD, "Unable to found m matrix");
        }
    }

    //if (m_publish_ros_pc) {ros_compute_and_send_pc(pc,m_ground_frame_id);}//<-------------------------

    yarp::sig::ImageOf<yarp::sig::PixelBgra>& imgOut = m_imgOutPort.prepare();
    imgOut.zero();

    m_floorMutex.lock();
    //compute the point cloud
    depthToFilteredPc();
    freeFloorDraw(imgOut);

    m_floorMutex.unlock();

    m_imgOutPort.write();
}

void FreeFloorThread::depthToFilteredPc()
{
    size_t max_x = m_pc_roi.max_x == 0 ? m_depth_image.width() : std::min(m_pc_roi.max_x,m_depth_image.width());
    size_t max_y = m_pc_roi.max_y == 0 ? m_depth_image.height() : std::min(m_pc_roi.max_y,m_depth_image.height());
    size_t min_x = std::min(m_pc_roi.min_x,max_x);
    size_t min_y = std::min(m_pc_roi.min_y,max_y);
    m_pc_stepx = std::max<size_t>(std::min(m_pc_stepx, max_x - min_x), 1);
    m_pc_stepy = std::max<size_t>(std::min(m_pc_stepy, max_y - min_y), 1);
    //yCInfo(FREE_FLOOR_THREAD, "Steps: x%d y%d",m_pc_stepx,m_pc_stepy);

    size_t size_x = (max_x-min_x)/m_pc_stepx;
    size_t size_y = (max_y-min_y)/m_pc_stepy;
    yarp::sig::Vector tempPoint(4,1.0);
    m_pc.clear();
    m_pc.resize(size_x,size_y);
    m_okPixels_pre.clear();
    m_obstacle_columns.clear();

    for (size_t u = m_pc_roi.min_x, i=0; u < max_x; u += m_pc_stepx, i++)
    {
        for (size_t v = m_pc_roi.min_y, j=0; v < max_y; v += m_pc_stepy, j++)
        {
            tempPoint[0] = (u - m_intrinsics.principalPointX) / m_intrinsics.focalLengthX * m_depth_image.pixel(u, v);
            tempPoint[1] = (v - m_intrinsics.principalPointY) / m_intrinsics.focalLengthY * m_depth_image.pixel(u, v);
            tempPoint[2] = m_depth_image.pixel(u, v);
            yarp::sig::Vector v2 = m_transform_mtrx*tempPoint;
            m_pc(i,j).x=v2(0);
            m_pc(i,j).y=v2(1);
            m_pc(i,j).z=v2(2);
            int xC = (int)(v2(0)*m_col_granularity);
            int yC = (int)(v2(1)*m_col_granularity);
            std::pair<int,int> tempKey(xC,yC);
            if(m_obstacle_columns.find(tempKey)==m_obstacle_columns.end())
            {
                m_obstacle_columns[tempKey] = (v2(2)>=m_floor_height && v2(2)<=m_ceiling_height) || v2(2)<0;
            }
            else
            {
                m_obstacle_columns[tempKey] = m_obstacle_columns[tempKey] || (v2(2)>=m_floor_height && v2(2)<=m_ceiling_height) || v2(2)<0;
            }
            if(v2(2)<m_floor_height && v2(2)>=0 && !m_obstacle_columns[tempKey])
            {
                std::pair<size_t,size_t> tempPair(u,v);
                m_okPixels_pre[tempPair] = tempKey;
            }
        }
    }
}

void FreeFloorThread::freeFloorDraw(yarp::sig::ImageOf<yarp::sig::PixelBgra> &output)
{
    output.resize(m_pc.width(),m_pc.height());
    yarp::sig::PixelBgra pOk(0,0,0,0.6);
    double arScaler = 0.2;
    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);
    yarp::os::Bottle* baseCmdIn = m_baseCmdInPort.read(false);
    bool navigating = currentStatus == yarp::dev::Nav2D::navigation_status_moving;
    bool moving = false;
    if (baseCmdIn){
        moving = (baseCmdIn->get(0).asFloat64() != 0.0) || (baseCmdIn->get(1).asFloat64() != 0.0);
    }
    //rotateAndCheck(m_pc, m_transform_mtrx,m_rgbImage,imgOut,m_okPixels,m_floor_height,m_ceiling_height);
    output.copy(m_rgbImage);

    m_okPixels.clear();
    for(auto const& blob : m_okPixels_pre){
        if(!m_obstacle_columns[blob.second]){
            int u = blob.first.first;
            int v = blob.first.second;
            m_okPixels.push_back(blob.first);
            pOk.r = (!navigating && moving) ? output.pixel(u,v).r*arScaler+255*(1-arScaler) : output.pixel(u,v).r*arScaler;
            pOk.b = (navigating) ? output.pixel(u,v).b*arScaler+255*(1-arScaler) : output.pixel(u,v).b*arScaler;
            pOk.g = (navigating || moving) ? output.pixel(u,v).g*arScaler : output.pixel(u,v).g*arScaler+255*(1-arScaler);
            output.pixel(u,v) = pOk;

        }
    }
}

void FreeFloorThread::onRead(yarp::os::Bottle &b)
{
    if(m_nav2DPoly.isValid())
    {
        yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
        if(m_iNav2D->getNavigationStatus(currentStatus))
        {
            if(currentStatus == yarp::dev::Nav2D::navigation_status_moving)
            {
                m_iNav2D->stopNavigation();
                return;
            }
        }
        else
        {
            yCError(FREE_FLOOR_THREAD,"An error occurred while retrieving the navigation status");
        }
    }

    yCInfo(FREE_FLOOR_THREAD,"Received: %s",b.toString().c_str());

    if(b.size() == 2)
    {
        reachSpot(b);
    }
    else if(b.size()==5){
        if(b.get(0).asString() != "base"){
            yCError(FREE_FLOOR_THREAD) << "The first element of the bottle should be \"base\" but it's actually:" << b.get(0).asString();
            return;
        }
        moveBase(b);
    }
    else if(b.size() == 4)
    {
        rotate(b);
    }
    else{
        yCError(FREE_FLOOR_THREAD,"The input bottle has the wrong number of elements");
    }
}

void FreeFloorThread::reachSpot(yarp::os::Bottle &b)
{
    size_t u = b.get(0).asInt32();
    if(u >= m_rgbImage.width() || u<0)
    {
        yCError(FREE_FLOOR_THREAD, "Pixel outside image boundaries");
    }
    size_t v = b.get(1).asInt32();
    if(v >= m_rgbImage.height() || v<0)
    {
        yCError(FREE_FLOOR_THREAD, "Pixel outside image boundaries");
    }
    m_floorMutex.lock();
    auto tempPC = yarp::sig::utils::depthToPC(m_depth_image, m_intrinsics,m_pc_roi,1,1);
    auto pixel = m_transform_mtrx*tempPC(u,v).toVector4();
    int xC = (int)(pixel(0)*m_col_granularity);
    int yC = (int)(pixel(1)*m_col_granularity);
    std::pair<int,int> tempCol(xC,yC);

    if(pixel(2)<m_floor_height && pixel(2)>=0 && !m_obstacle_columns[tempCol])
    {
        if(m_nav2DPoly.isValid())
        {
            if(m_self_reliant) {m_iNav2D->gotoTargetByRelativeLocation(pixel(0),pixel(1));}
            else
            {
                double theta = atan2(pixel(1),pixel(0))*180.0/M_PI;
                pixel = m_transform_mtrx_extern*tempPC(u,v).toVector4();
                yarp::dev::Nav2D::Map2DLocation locPixel;
                m_iNav2D->getCurrentPosition(locPixel);
                locPixel.x = pixel(0);
                locPixel.y = pixel(1);
                locPixel.theta += theta;
                m_iNav2D->gotoTargetByAbsoluteLocation(locPixel);
            }
        }
        yarp::os::Bottle& toSend = m_targetOutPort.prepare();
        toSend.clear();
        toSend.addFloat32(pixel(0));
        toSend.addFloat32(pixel(1));
        m_targetOutPort.write();
    }
    m_floorMutex.unlock();
}

void FreeFloorThread::rotate(yarp::os::Bottle &b)
{
    double notNeeded, horizFOV;
    bool fovGot = m_iRgbd->getRgbFOV(horizFOV,notNeeded);
    if(!fovGot)
    {
        yCError(FREE_FLOOR_THREAD,"An error occurred while retrieving the rgb camera FOV");
    }

    int deltaPx = (b.get(0).asInt32()-b.get(2).asInt32());
    //double rotation = (double)deltaPx * horizFOV/m_depth_width;
    double rotation = (double)deltaPx * 180.0/m_depth_width;
    //int sign = (b.get(0).asInt32()-b.get(2).asInt32())/(abs(b.get(0).asInt32()-b.get(2).asInt32())>0?abs(b.get(0).asInt32()-b.get(2).asInt32()):1);
    if(m_nav2DPoly.isValid())
    {
        if(m_self_reliant) {m_iNav2D->gotoTargetByRelativeLocation(0.0,0.0,rotation);}
        else
        {
            yarp::dev::Nav2D::Map2DLocation locPixel;
            m_iNav2D->getCurrentPosition(locPixel);
            locPixel.theta += rotation;
            m_iNav2D->gotoTargetByAbsoluteLocation(locPixel);
        }
    }
}

void FreeFloorThread::moveBase(yarp::os::Bottle& b)
{
    bool leftRight = false;
    bool upDown = false;
    if(b.get(1).asInt32() != 0 && b.get(2).asInt32() == 0){
        m_outputBaseData.vel_theta = (double)b.get(1).asInt32()*(m_maxVelTheta)/100.0;
    }
    else if(b.get(2).asInt32() != 0 && b.get(1).asInt32() == 0){
        m_outputBaseData.vel_theta = (double)b.get(2).asInt32()*(m_maxVelTheta)/100.0*(-1.0);
    }
    else if(b.get(2).asInt32() != 0 && b.get(1).asInt32() != 0){
        yCError(FREE_FLOOR_THREAD) << "You cannot go both left and right";
        leftRight = true;
    }
    else{
        m_outputBaseData.vel_theta = 0.0;
    }
    if(b.get(3).asInt32() != 0 && b.get(4).asInt32() == 0){
        m_outputBaseData.vel_x = (double)b.get(3).asInt32()*(m_maxVelX)/100.0;
    }
    else if(b.get(4).asInt32() != 0 && b.get(3).asInt32() == 0){
        m_outputBaseData.vel_x = (double)b.get(2).asInt32()*(m_maxVelX)/100.0*(-1.0);
    }
    else if(b.get(4).asInt32() != 0 && b.get(3).asInt32() != 0){
        yCError(FREE_FLOOR_THREAD) << "You cannot go both forward and backward";
        upDown = true;
    }
    else{
        m_outputBaseData.vel_x = 0.0;
    }
    if(upDown || leftRight){
        return;
    }
    m_baseCmdOutPort.write(m_outputBaseData);
    m_currentVelTheta = m_outputBaseData.vel_theta;
    m_currentVelX = m_outputBaseData.vel_x;
}

void FreeFloorThread::threadRelease()
{
#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "Thread releasing...");
#endif

    if(m_rgbdPoly.isValid())
        m_rgbdPoly.close();

    if(m_tcPoly.isValid())
        m_tcPoly.close();
    if(m_nav2DPoly.isValid())
        m_nav2DPoly.close();
    if(!m_imgOutPort.isClosed()){
        m_imgOutPort.close();
    }
    if(!m_targetOutPort.isClosed()){
        m_targetOutPort.close();
    }

    yCInfo(FREE_FLOOR_THREAD, "Thread released");

#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "... done.");
#endif

    return;
}
