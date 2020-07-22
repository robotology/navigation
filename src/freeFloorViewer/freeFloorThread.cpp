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

yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>* pointCloud_outTopic = nullptr;
yarp::os::Node                                             * rosNode=nullptr;
/*
void ros_init_pc()
{
    //  rosNode = new yarp::os::Node("laserFromPointCloud");

    pointCloud_outTopic=new yarp::os::Publisher<yarp::rosmsg::sensor_msgs::PointCloud2>;
    if (pointCloud_outTopic->topic("/ros_pc")==false)
    {
        yCError(FREE_FLOOR_THREAD,"opening topic");
    }
    else
    {
        yCInfo(FREE_FLOOR_THREAD,"topic successful");
    }
}

void ros_compute_and_send_pc(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, std::string frame_id)
{
    yarp::rosmsg::sensor_msgs::PointCloud2               rosPC_data;
    static int counter=0;
    rosPC_data.header.stamp.nsec=0;
    rosPC_data.header.stamp.sec=0;
    rosPC_data.header.seq=counter++;
    rosPC_data.header.frame_id = frame_id;

    rosPC_data.fields.resize(3);
    rosPC_data.fields[0].name       = "x";
    rosPC_data.fields[0].offset     = 0;    // offset in bytes from start of each point
    rosPC_data.fields[0].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[0].count      = 1;    // how many FLOAT32 used for 'x'

    rosPC_data.fields[1].name       = "y";
    rosPC_data.fields[1].offset     = 4;    // offset in bytes from start of each point
    rosPC_data.fields[1].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[1].count      = 1;    // how many FLOAT32 used for 'y'

    rosPC_data.fields[2].name       = "z";
    rosPC_data.fields[2].offset     = 8;    // offset in bytes from start of each point
    rosPC_data.fields[2].datatype   = 7;    // 7 = FLOAT32
    rosPC_data.fields[2].count      = 1;    // how many FLOAT32 used for 'z'

#if defined(YARP_BIG_ENDIAN)
    rosPC_data.is_bigendian = true;
#elif defined(YARP_LITTLE_ENDIAN)
    rosPC_data.is_bigendian = false;
#else
    #error "Cannot detect endianness"
#endif

#if 0
    rosPC_data.height=1;
    rosPC_data.width=pc.size();
#else
    rosPC_data.height=pc.height();
    rosPC_data.width=pc.width();
#endif

    rosPC_data.point_step = 3*4; //x, y, z
    rosPC_data.row_step   = rosPC_data.point_step*rosPC_data.width; //12 *number of points bytes
    rosPC_data.is_dense = true;   // what this field actually means?? When is it false??
    rosPC_data.data.resize(rosPC_data.row_step*rosPC_data.height);

    const char* ypointer = pc.getRawData()+12;
    unsigned char* rpointer = rosPC_data.data.data();

    size_t elem =0;
    size_t yelem=0;
    for (; elem<pc.size()*3*4; elem++)
    {
        *rpointer=*ypointer;
        rpointer++;
        ypointer++; yelem++;
        if (elem%12==0) { ypointer+=4; yelem+=4;}
    }

    if (pointCloud_outTopic) pointCloud_outTopic->write(rosPC_data);
}
*/
void rotate_pc (yarp::sig::PointCloud<yarp::sig::DataXYZ>& pc, const yarp::sig::Matrix& m)
{
    for (size_t i=0; i<pc.size(); i++)
    {
        auto v1 = pc(i).toVector4();
        auto v2 = m*v1;
        pc(i).x=v2(0);
        pc(i).y=v2(1);
        pc(i).z=v2(2);
    }
}

void checkPcZ(const yarp::sig::PointCloud<yarp::sig::DataXYZ>& inputPc,
              const yarp::sig::FlexImage& inputCanvas,
              yarp::sig::ImageOf<yarp::sig::PixelBgra> &output,
              double threshold)
{
    yCInfo(FREE_FLOOR_THREAD,"Pixel code inputCanvas:\t%d\t%d",inputCanvas.getPixelCode(),inputCanvas.getPixelSize());
    output.resize(inputPc.width(),inputPc.height());
    yarp::sig::PixelBgra pOk;
    pOk.g = 255;
    pOk.r = 0;
    pOk.b = 0;
    pOk.a = 0.6;
    yarp::sig::PixelBgra pKo;
    pKo.g = 0;
    pKo.r = 0;
    pKo.b = 0;
    pKo.a = 0;

    auto* inPixels = reinterpret_cast<yarp::sig::PixelBgra *> (inputCanvas.getRawImage());

    for (size_t r=0; r<inputPc.height(); r++)
    {
        for(size_t c=0; c<inputPc.width();c++){
            if(inputPc(c,r).z <= threshold){
                output.pixel(c,r) = pOk;
            }
            //else {output.pixel(c,r) = pKo;}
            else{
                output.pixel(c,r) = inPixels[c+r*inputCanvas.width()];
                yarp::sig::PixelBgra x = inPixels[c+r*inputCanvas.width()];
            }
        }
    }
}

FreeFloorThread::FreeFloorThread(double _period, yarp::os::ResourceFinder &rf):
    PeriodicThread(_period),
    m_rf(rf)
{
    m_depth_width = 0;
    m_depth_height = 0;
    m_pc_stepx = 2;
    m_pc_stepy = 2;
    m_pointcloud_max_distance = 10;
    m_floor_height = 0.1;
    m_ground_frame_id = "/ground_frame";
    m_camera_frame_id = "/depth_camera_frame";
    m_imgOutPortName = "/freeFloorViewer/floorEnhanced:o";

    printed = false;
}

bool FreeFloorThread::threadInit()
{
#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "thread initialising...\n");
#endif

    // --------- Generic config --------- //
    m_publish_ros_pc = m_rf.check("publish_ROS_pointcloud");
    bool okZClipRf = m_rf.check("Z_CLIPPING_PLANES");
    if(okZClipRf){
        yarp::os::Searchable& pointcloud_clip_config = m_rf.findGroup("Z_CLIPPING_PLANES");
        if (pointcloud_clip_config.check("floor_height"))   {m_floor_height = pointcloud_clip_config.find("floor_height").asFloat64();}
        if (pointcloud_clip_config.check("max_distance")) { m_pointcloud_max_distance = pointcloud_clip_config.find("max_distance").asFloat64(); }
        if (pointcloud_clip_config.check("ground_frame_id")) {m_ground_frame_id = pointcloud_clip_config.find("ground_frame_id").asString();}
        if (pointcloud_clip_config.check("camera_frame_id")) {m_camera_frame_id = pointcloud_clip_config.find("camera_frame_id").asString();}
    }

    // --------- RGBDSensor config --------- //
    bool okRgbdRf = m_rf.check("RGBD_SENSOR_CLIENT");
    if(!okRgbdRf){
        yCError(FREE_FLOOR_THREAD,"RGBD_SENSOR_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property rgbdProp;
    rgbdProp.fromString(m_rf.findGroup("RGBD_SENSOR_CLIENT").toString());
    m_rgbdPoly.open(rgbdProp);
    if(!m_rgbdPoly.isValid()){
        yCError(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_rgbdPoly.view(m_iRgbd);
    if(!m_iRgbd){
        yCError(FREE_FLOOR_THREAD,"Error opening iRGBD interface. Device not available");
        return false;
    }
    //Verify if this is needed
    yarp::os::Time::delay(0.1);

    // --------- TransformClient config --------- //
    bool okTransformRf = m_rf.check("TRANSFORM_CLIENT");
    if(!okTransformRf){
        yCError(FREE_FLOOR_THREAD,"TRANSFORM_CLIENT section missing in ini file");

        return false;
    }
    yarp::os::Property tcProp;
    tcProp.fromString(m_rf.findGroup("TRANSFORM_CLIENT").toString());
    m_tcPoly.open(tcProp);
    if(!m_tcPoly.isValid()){
        yCError(FREE_FLOOR_THREAD,"Error opening PolyDriver check parameters");
        return false;
    }
    m_tcPoly.view(m_iTc);
    if(!m_iTc){
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
    m_imgOutSandro.open("/sandro");

    /*if (m_publish_ros_pc)
    {
       ros_init_pc();
    }*/

#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "... done!\n");
#endif

    return true;
}

void FreeFloorThread::run()
{
    std::lock_guard<std::mutex> lock(m_floorMutex);
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
    yCInfo(FREE_FLOOR_THREAD,"Pixel code m_rgbdImage:\t%d",m_rgbImage.getPixelCode());
    yCInfo(FREE_FLOOR_THREAD,"Pixel code m_rgbdImage:\t%d\t%d",m_rgbImage.width(),m_rgbImage.height());
    if(!printed){
        yCInfo(FREE_FLOOR_THREAD,"\n\n# ---------------- IMAGE START ---------------- #\n");
        for(int r=0; r<m_rgbImage.height();r++){
            auto row = m_rgbImage.getRow(r);
            for(int c=0; c<m_rgbImage.width(); c++){
                int cabella = row[c]+'0';
                printf("\t%d",cabella);
            }
            printf("\n");
        }
        yCInfo(FREE_FLOOR_THREAD,"\n# ---------------- IMAGE END ------------------ #\n");
        printed = true;
    }

    if (m_depth_image.width()!=m_depth_width ||
        m_depth_image.height()!=m_depth_height)
    {
        yCDebug(FREE_FLOOR_THREAD,"invalid image size: (%d %d) vs (%d %d)",m_depth_image.width(),m_depth_image.height(),m_depth_width,m_depth_height);
        return;
    }

    //compute the point cloud
    yarp::sig::PointCloud<yarp::sig::DataXYZ> pc = yarp::sig::utils::depthToPC(m_depth_image, m_intrinsics,m_pc_roi,m_pc_stepx,m_pc_stepy);

    //we compute the transformation matrix from the camera to the laser reference frame

    bool frame_exists = m_iTc->getTransform(m_camera_frame_id,m_ground_frame_id, m_transform_mtrx);
    if (frame_exists==false)
    {
        yCWarning(FREE_FLOOR_THREAD, "Unable to found m matrix");
    }

    //we rototranslate the full pointcloud
    rotate_pc(pc, m_transform_mtrx);

    //if (m_publish_ros_pc) {ros_compute_and_send_pc(pc,m_ground_frame_id);}//<-------------------------

    yarp::sig::ImageOf<yarp::sig::PixelBgra>& imgOut = m_imgOutPort.prepare();
    imgOut.zero();

    checkPcZ(pc,m_rgbImage,imgOut,m_floor_height);

    m_imgOutPort.write();
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
    if(!m_imgOutPort.isClosed()){
        m_imgOutPort.close();
    }
    if(!m_imgOutSandro.isClosed()){
        m_imgOutSandro.close();
    }

    yCInfo(FREE_FLOOR_THREAD, "Thread released");

#ifdef FREEFLOOR_DEBUG
    yCDebug(FREE_FLOOR_THREAD, "... done.");
#endif

    return;
}
