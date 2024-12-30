/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Map2DLocation.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <mutex>
#include <math.h>
#include "navigation_defines.h"

using namespace yarp::os;

class movable_localization_device
{
private:

    //device position
    enum
    {
        DEVICE_POS_IS_NONE = 0,
        DEVICE_POS_IS_FIXED = 1,
        DEVICE_FROM_TF_FIXED = 2,
        DEVICE_FROM_TF_VARIABLE = 3,
    }                            m_device_position;

    //configuration options


    //tf data
    yarp::dev::PolyDriver        m_ptf;
    yarp::dev::IFrameTransform*  m_iTf;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_device_id;
    
    //fixed position
    yarp::dev::Nav2D::Map2DLocation     m_device_to_robot_transform;
    
    

public:
    movable_localization_device();
    bool init(const yarp::os::Searchable& cfg, yarp::dev::IFrameTransform*  iTf=nullptr);
    void relocate_data(yarp::dev::Nav2D::Map2DLocation& device_location);
    
private:
    bool init_tf();

};
