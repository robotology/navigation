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
    m_imgOutPortName = "/freeFloorViewer/floorEnhanced:o";
    m_targetOutPortName = "/free_floor_viewer/target:o";
}

bool FreeFloorThread::threadInit()
{
#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
    if(m_rf.check("target_pos_port")) {m_targetOutPortName = m_rf.find("target_pos_port").asString();}
    if(m_rf.check("img_out_port")) {m_imgOutPortName = m_rf.find("img_out_port").asString();}

    // --------- Z related props -------- //
    bool okZClipRf = m_rf.check("Z_CLIPPING_PLANES");
    if(okZClipRf)
    {
        yarp::os::Searchable& pointcloud_clip_config = m_rf.findGroup("Z_CLIPPING_PLANES");
        if (pointcloud_clip_config.check("floor_height"))   {m_floor_height = pointcloud_clip_config.find("floor_height").asFloat64();}
        if (pointcloud_clip_config.check("ceiling_height"))   {m_ceiling_height = pointcloud_clip_config.find("ceiling_height").asFloat64();}
        if (pointcloud_clip_config.check("ground_frame_id")) {m_ground_frame_id = pointcloud_clip_config.find("ground_frame_id").asString();}
        if (pointcloud_clip_config.check("camera_frame_id")) {m_camera_frame_id = pointcloud_clip_config.find("camera_frame_id").asString();}
        if (pointcloud_clip_config.check("column_granularity")) {m_col_granularity = pointcloud_clip_config.find("column_granularity").asInt();}
    }

    // --------- Point cloud quality -------- //
    bool okPCQuality = m_rf.check("POINTCLOUD_QUALITY");
    if(okPCQuality)
    {
        yarp::os::Searchable& pointcloud_qual_config = m_rf.findGroup("POINTCLOUD_QUALITY");
        if (pointcloud_qual_config.check("x_step"))   {m_pc_stepx = pointcloud_qual_config.find("x_step").asInt();}
        if (pointcloud_qual_config.check("y_step"))   {m_pc_stepy = pointcloud_qual_config.find("y_step").asInt();}
    }

    // --------- RGBDSensor config --------- //
    bool okRgbdRf = m_rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf)
    {
        yCError(FREE_FLOOR_THREAD,"RGBD_SENSOR_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property rgbdProp;
    rgbdProp.fromString(m_rf.findGroup("RGBD_SENSOR_CLIENT").toString());
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

    // --------- TransformClient config --------- //
    bool okTransformRf = m_rf.check("TRANSFORM_CLIENT");
    if(!okTransformRf)
    {
        yCError(FREE_FLOOR_THREAD,"TRANSFORM_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property tcProp;
    tcProp.fromString(m_rf.findGroup("TRANSFORM_CLIENT").toString());
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
    bool okNavigation2DRf = m_rf.check("NAVIGATION_CLIENT");
    if(!okNavigation2DRf)
    {
        yCError(FREE_FLOOR_THREAD,"NAVIGATION_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property nav2DProp;
    nav2DProp.fromString(m_rf.findGroup("NAVIGATION_CLIENT").toString());
    m_nav2DPoly.open(nav2DProp);
    if(!m_nav2DPoly.isValid())
    {
        yCError(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters");
        return false;
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

    m_imgOutPort.open(m_imgOutPortName);
    m_targetOutPort.open(m_targetOutPortName);

#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "... done!\n");
#endif

    return true;
}

void FreeFloorThread::run()
{
    //std::lock_guard<std::mutex> lock(m_floorMutex);
    bool depth_ok = m_iRgbd->getDepthImage(m_depth_image);
    if (depth_ok == false)
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
    if (rgb_ok == false)
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
    if (frame_exists==false)
    {
        yCWarning(FREE_FLOOR_THREAD, "Unable to found m matrix");
    }

    //if (m_publish_ros_pc) {ros_compute_and_send_pc(pc,m_ground_frame_id);}//<-------------------------

    yarp::sig::ImageOf<yarp::sig::PixelBgra>& imgOut = m_imgOutPort.prepare();
    imgOut.zero();

    m_floorMutex.lock();
    //compute the point cloud
    auto t1 = std::chrono::high_resolution_clock::now();
    depthToFilteredPc();
    auto t2 = std::chrono::high_resolution_clock::now();
    freeFloorDraw(imgOut);
    auto t3 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>( t3 - t2 ).count();
    yCInfo(FREE_FLOOR_THREAD) << "The durations are: " << duration << " " << duration2;

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
    yCInfo(FREE_FLOOR_THREAD, "Steps: x%d y%d",m_pc_stepx,m_pc_stepy);

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
    bool moving = currentStatus == yarp::dev::Nav2D::navigation_status_moving;
    //rotateAndCheck(m_pc, m_transform_mtrx,m_rgbImage,imgOut,m_okPixels,m_floor_height,m_ceiling_height);
    output.copy(m_rgbImage);

    m_okPixels.clear();
    for(auto const& blob : m_okPixels_pre){
        if(!m_obstacle_columns[blob.second]){
            int u = blob.first.first;
            int v = blob.first.second;
            m_okPixels.push_back(blob.first);
            pOk.r = moving ? output.pixel(u,v).r*arScaler+255*(1-arScaler) : output.pixel(u,v).r*arScaler;
            pOk.b = output.pixel(u,v).b*arScaler;
            pOk.g = moving ? output.pixel(u,v).g*arScaler : output.pixel(u,v).g*arScaler+255*(1-arScaler);
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
    if(b.size() == 2)
    {
        reachSpot(b);
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
    size_t u = b.get(0).asInt();
    if(u >= m_rgbImage.width() || u<0)
    {
        yCError(FREE_FLOOR_THREAD, "Pixel outside image boundaries");
    }
    size_t v = b.get(1).asInt();
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
            m_iNav2D->gotoTargetByRelativeLocation(pixel(0),pixel(1));
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

    int deltaPx = (b.get(0).asInt()-b.get(2).asInt());
    double rotation = (double)deltaPx * horizFOV/m_depth_width;
    //int sign = (b.get(0).asInt()-b.get(2).asInt())/(abs(b.get(0).asInt()-b.get(2).asInt())>0?abs(b.get(0).asInt()-b.get(2).asInt()):1);
    if(m_nav2DPoly.isValid())
    {
        m_iNav2D->gotoTargetByRelativeLocation(0.0,0.0,rotation);
    }
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
