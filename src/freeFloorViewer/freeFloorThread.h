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

#ifndef FREE_FLOOR_THREAD_H
#define FREE_FLOOR_THREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/sig/IntrinsicParams.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/sig/PointCloudUtils.h>
#include <yarp/math/Math.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <math.h>
#include <mutex>
#include <algorithm>


class FreeFloorThread : public yarp::os::PeriodicThread, public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
protected:
    //Devices related attributes
    yarp::dev::PolyDriver            m_rgbdPoly;
    yarp::dev::IRGBDSensor*          m_iRgbd{nullptr};
    yarp::dev::PolyDriver            m_tcPoly;
    yarp::dev::IFrameTransform*      m_iTc{nullptr};
    yarp::dev::PolyDriver            m_nav2DPoly;
    yarp::dev::Nav2D::INavigation2D* m_iNav2D{nullptr};

    //Computation related attributes
    bool   m_publish_ros_pc;
    int m_depth_width;
    int m_depth_height;
    double m_floor_height;
    double m_ceiling_height;
    size_t m_pc_stepx;
    size_t m_pc_stepy;
    std::string m_ground_frame_id;
    std::string m_camera_frame_id;
    std::vector<std::pair<size_t,size_t>> m_okPixels;
    yarp::os::Property m_propIntrinsics;
    yarp::sig::IntrinsicParams m_intrinsics;
    yarp::sig::ImageOf<float> m_depth_image;
    yarp::sig::FlexImage m_rgbImage;
    yarp::sig::utils::PCL_ROI m_pc_roi;
    yarp::sig::Matrix m_transform_mtrx;
    yarp::sig::PointCloud<yarp::sig::DataXYZ> m_pc;

    //Ports
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgra>> m_imgOutPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_targetOutPort;
    std::string m_imgOutPortName;
    std::string m_targetOutPortName;

    //Others
    yarp::os::ResourceFinder &m_rf;
    std::mutex pixelLocker;

public:
    //Public attributes
    std::mutex m_floorMutex;

    //Contructor and distructor
    FreeFloorThread(double _period, yarp::os::ResourceFinder &rf);
    ~FreeFloorThread()
    {
    }

    //methods inherited from PeriodicThread
    virtual void run() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;

    //Internal methods
    void reachSpot(yarp::os::Bottle& b);
    void rotate(yarp::os::Bottle& b);
    void rotateAndCheck(yarp::sig::ImageOf<yarp::sig::PixelBgra> &output);

    //Port callback
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& b) override;
};

#endif
