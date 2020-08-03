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


#define _USE_MATH_DEFINES

#include "freeFloorThread.h"

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
    m_pc = yarp::sig::utils::depthToPC(m_depth_image, m_intrinsics,m_pc_roi,m_pc_stepx,m_pc_stepy);
    rotateAndCheck(imgOut);
    m_floorMutex.unlock();

    m_imgOutPort.write();
}

void FreeFloorThread::rotateAndCheck(yarp::sig::ImageOf<yarp::sig::PixelBgra> &output)
{
    output.resize(m_pc.width(),m_pc.height());
    yarp::sig::PixelBgra pOk(0,0,0,0.6);
    std::map<std::pair<int,int>,bool> columns;
    int scaler = 10;
    double arScaler = 0.5;
    yarp::dev::Nav2D::NavigationStatusEnum currentStatus;
    m_iNav2D->getNavigationStatus(currentStatus);
    bool moving = currentStatus == yarp::dev::Nav2D::navigation_status_moving;
    //rotateAndCheck(m_pc, m_transform_mtrx,m_rgbImage,imgOut,m_okPixels,m_floor_height,m_ceiling_height);
    output.copy(m_rgbImage);

    m_okPixels.clear();
    for (size_t r=0; r<m_pc.height(); r++)
    {
        for(size_t c=0; c<m_pc.width();c++)
        {
            auto v1 = m_pc(c,r).toVector4();
            auto v2 = m_transform_mtrx*v1;
            m_pc(c,r).x=v2(0);
            m_pc(c,r).y=v2(1);
            m_pc(c,r).z=v2(2);
            int xC = (int)(v2(0)*scaler);
            int yC = (int)(v2(1)*scaler);
            std::pair<int,int> tempKey(xC,yC);
            if(columns.count(tempKey)==0)
            {
                columns[tempKey] = (v2(2)>=m_floor_height && v2(2)<=m_ceiling_height) || v2(2)<0;
            }
            if(v2(2)<m_floor_height && v2(2)>=0 && !columns[tempKey])
            {
                std::pair<size_t,size_t> tempPair;
                tempPair.first = c;
                tempPair.second = r;
                m_okPixels.push_back(tempPair);
                pOk.r = moving ? output.pixel(c,r).r*arScaler+255*(1-arScaler) : output.pixel(c,r).r;
                pOk.b = output.pixel(c,r).b;
                pOk.g = moving ? output.pixel(c,r).g : output.pixel(c,r).g*arScaler+255*(1-arScaler);
                output.pixel(c,r) = pOk;
            }
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
    std::pair<size_t,size_t> gotPos;
    size_t u = b.get(0).asInt();
    if(u >= m_rgbImage.width() or u<0)
    {
        yCError(FREE_FLOOR_THREAD, "Pixel outside image boundaries");
    }
    size_t v = b.get(1).asInt();
    if(v >= m_rgbImage.height() or v<0)
    {
        yCError(FREE_FLOOR_THREAD, "Pixel outside image boundaries");
    }
    gotPos.first = u;
    gotPos.second = v;
    m_floorMutex.lock();
    bool pixelOk = std::find(m_okPixels.begin(), m_okPixels.end(), gotPos) != m_okPixels.end();

    if(pixelOk)
    {
        if(m_nav2DPoly.isValid())
        {
            m_iNav2D->gotoTargetByRelativeLocation(m_pc(u,v).x,m_pc(u,v).y);
        }
        yarp::os::Bottle& toSend = m_targetOutPort.prepare();
        toSend.clear();
        toSend.addFloat32(m_pc(u,v).x);
        toSend.addFloat32(m_pc(u,v).y);
        m_targetOutPort.write();
    }
    m_floorMutex.unlock();
}

void FreeFloorThread::rotate(yarp::os::Bottle &b)
{
    yCInfo(FREE_FLOOR_THREAD,"The bottle: [%d, %d, %d, %d]",b.get(0).asInt(),b.get(1).asInt(),
           b.get(2).asInt(),b.get(3).asInt());

    double notNeeded, horizFOV;
    bool fovGot = m_iRgbd->getRgbFOV(horizFOV,notNeeded);
    if(!fovGot)
    {
        yCError(FREE_FLOOR_THREAD,"An error occurred while retrieving the rgb camera FOV");
    }

    yCInfo(FREE_FLOOR_THREAD,"FOV: [%f, %f]",horizFOV,notNeeded);

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
