/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include "movable_localization_device.h"
#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef RAD2DEG
#define RAD2DEG 180/M_PI
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif

YARP_LOG_COMPONENT(MOVABLE_DEV, "navigation.navigation_lib.movable_localization_device")

//////////////////////////

void movable_localization_device::relocate_data(Map2DLocation& device_location)
{
    if (m_device_position==DEVICE_POS_IS_NONE)
    {
        return;
    }
    else if (m_device_position == DEVICE_POS_IS_FIXED)
    {
        device_location.x     += m_device_to_robot_transform.x;
        device_location.y     += m_device_to_robot_transform.y;
        device_location.theta += m_device_to_robot_transform.theta;
    }
    else if (m_device_position == DEVICE_FROM_TF_FIXED)
    {
        device_location.x += m_device_to_robot_transform.x;
        device_location.y += m_device_to_robot_transform.y;
        device_location.theta += m_device_to_robot_transform.theta;
    }
    else if (m_device_position == DEVICE_FROM_TF_VARIABLE)
    {
        yarp::sig::Matrix transform;
        bool b = m_iTf->getTransform(m_frame_robot_id, m_frame_device_id, transform);
        m_device_to_robot_transform.x = transform[3][0];
        m_device_to_robot_transform.y = transform[3][1];
        auto v = yarp::math::dcm2euler(transform);
        m_device_to_robot_transform.theta = v[2] * RAD2DEG;
    }
}


movable_localization_device::movable_localization_device()
{
    //default values
    m_iTf = nullptr;
    m_frame_robot_id = "robot_frame";
    m_frame_device_id = "device_frame";
    m_tf_data_received = 0;
    m_device_position = DEVICE_POS_IS_NONE;
}

bool movable_localization_device::init(const yarp::os::Searchable&  cfg, yarp::dev::IFrameTransform*  iTf)
{
    if (m_iTf == nullptr)
    {
        m_iTf = iTf;
    }
    else
    {
        yCWarning(MOVABLE_DEV) << "iTf already exists";
        m_iTf = iTf;
    }

    Bottle devicepos_group = cfg.findGroup("DEVICE_POS");
    if (devicepos_group.isNull())
    {
        yCError(MOVABLE_DEV) << "Missing DEVICE_POS group!";
        return false;
    }

    //device position initialization
    if (devicepos_group.check("device_position"))
    {
        std::string device_position_mode = devicepos_group.find("device_position").asString();
        if (device_position_mode == "fixed") m_device_position = DEVICE_POS_IS_FIXED;
        else if (device_position_mode == "from_tf_fixed") m_device_position = DEVICE_FROM_TF_FIXED;
        else if (device_position_mode == "from_tf_variable") m_device_position = DEVICE_FROM_TF_VARIABLE;
        else { yCError(MOVABLE_DEV) << "Invalid value for 'device_position' param"; return false; }
    }
    else { yCError(MOVABLE_DEV) << "missing 'device_position' param"; return false; }

    if (m_device_position == DEVICE_POS_IS_FIXED)
    {
        m_device_to_robot_transform.x = devicepos_group.find("device_position_x").asFloat64();
        m_device_to_robot_transform.y = devicepos_group.find("device_position_y").asFloat64();
        m_device_to_robot_transform.theta = devicepos_group.find("device_position_t").asFloat64();
    }
    else if (m_device_position == DEVICE_FROM_TF_FIXED)
    {
        if (m_iTf == nullptr)
        {
            if (!init_tf())  { yCError(MOVABLE_DEV) << "general error"; return false; }
        }
        yarp::sig::Matrix transform;
        bool b = m_iTf->getTransform(m_frame_robot_id,m_frame_device_id,transform);
        m_device_to_robot_transform.x = transform[3][0];
        m_device_to_robot_transform.y = transform[3][1];
        auto v = yarp::math::dcm2euler(transform);
        m_device_to_robot_transform.theta = v[2] * RAD2DEG;
    }
    else if (m_device_position == DEVICE_FROM_TF_VARIABLE)
    {
        if (m_iTf == nullptr)
        {
            if (!init_tf()) { yCError(MOVABLE_DEV) << "m_iTf is nullptr"; return false; }
        }
    }
    else
    {
        yCError(MOVABLE_DEV) << "m_device_position type unset";
        return false;
    }
}

bool movable_localization_device::init_tf()
{
    std::string module_name;
    Property options;
    options.put("device", TF_CLIENT_DEFAULT_DEVICE);
    options.put("local", "/" + module_name + "/TfClient");
    options.put("remote", "/transformServer");
    if (m_ptf.open(options) == false)
    {
        yCError(MOVABLE_DEV) << "Unable to open transform client";
        return false;
    }
    m_ptf.view(m_iTf);
    if (m_ptf.isValid() == false || m_iTf == nullptr)
    {
        yCError(MOVABLE_DEV) << "Unable to view iTransform interface";
        return false;
    }
    return true;
}