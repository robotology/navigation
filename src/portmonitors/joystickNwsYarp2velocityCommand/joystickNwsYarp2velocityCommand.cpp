/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "joystickNwsYarp2velocityCommand.h"

#include <yarp/dev/AllJoyData.h>
#include <yarp/dev/StickDataList.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

YARP_LOG_COMPONENT(JOY2VEL, "navigation.JoyNwsYarp2Vel")

namespace
{
constexpr double vtheta_scale = 30.0;
constexpr int metaquest_bottle_size = 4;

bool isNumeric(yarp::os::Value& val)
{
    if (val.isFloat32() ||
        val.isFloat64() ||
        val.isInt8() ||
        val.isInt16() ||
        val.isInt32() ||
        val.isInt64())
    {
        return true;
    }
    return false;
}
}

JoyNwsYarp2vel::JoyNwsYarp2vel()
{
    this->m_things.setPortWriter(&this->m_command);
}

bool JoyNwsYarp2vel::accept(yarp::os::Things& thing)
{
    if (yarp::dev::AllJoyData* data = thing.cast_as<yarp::dev::AllJoyData>())
    {
        return validate_all_joy_data(data);
    }

    if (yarp::dev::StickDataList* data = thing.cast_as<yarp::dev::StickDataList>())
    {
        return validate_stick_data_list(data);
    }

    if (yarp::os::Bottle* bot = thing.cast_as<yarp::os::Bottle>())
    {
        return validate_metaquest_bot(bot);
    }

    yCWarning(JOY2VEL, "expected type AllJoyData, StickDataList or Bottle but got wrong data type!");
    return false;
}

bool JoyNwsYarp2vel::validate_all_joy_data(const yarp::dev::AllJoyData* data)
{
    if (!data)
    {
        yCError(JOY2VEL, "Invalid AllJoyData message: empty message");
        return false;
    }

    if (data->StickDataVal.empty())
    {
        yCError(JOY2VEL, "Invalid AllJoyData message: missing stick data");
        return false;
    }

    return true;
}

bool JoyNwsYarp2vel::validate_stick_data_list(const yarp::dev::StickDataList* data)
{
    if (!data)
    {
        yCError(JOY2VEL, "Invalid StickDataList message: empty message");
        return false;
    }

    if (data->value.empty())
    {
        yCError(JOY2VEL, "Invalid StickDataList message: missing stick data");
        return false;
    }

    return true;
}

bool JoyNwsYarp2vel::validate_metaquest_bot(const yarp::os::Bottle* bot)
{
    if (!bot)
    {
        yCError(JOY2VEL, "Invalid Meta Quest bottle format: empty bottle");
        return false;
    }

    if (bot->size() != metaquest_bottle_size)
    {
        yCError(JOY2VEL, "Invalid Meta Quest bottle format: expected 4 values");
        return false;
    }

    // Meta Quest messages are expected as:
    // [left_x, left_y, right_x, right_y]
    for (int i = 0; i < metaquest_bottle_size; ++i)
    {
        yarp::os::Value value = bot->get(i);
        if (!isNumeric(value))
        {
            yCError(JOY2VEL, "Invalid Meta Quest bottle format: item %d is not numeric", i);
            return false;
        }
    }

    return true;
}

void JoyNwsYarp2vel::update_command(double x, double y, double theta)
{
    m_command.vel_x = x;
    m_command.vel_y = y;
    m_command.vel_theta = theta;
}

yarp::os::Things& JoyNwsYarp2vel::update(yarp::os::Things& thing)
{
    if (yarp::dev::AllJoyData* data = thing.cast_as<yarp::dev::AllJoyData>())
    {
        yAssert(validate_all_joy_data(data));

        const auto& left_stick = data->StickDataVal[0];
        const double right_x = (data->StickDataVal.size() > 1) ? data->StickDataVal[1].s1 : 0.0;
        update_command(left_stick.s2, left_stick.s1, -right_x * vtheta_scale);
        return this->m_things;
    }

    if (yarp::dev::StickDataList* data = thing.cast_as<yarp::dev::StickDataList>())
    {
        yAssert(validate_stick_data_list(data));

        const auto& left_stick = data->value[0];
        const double right_x = (data->value.size() > 1) ? data->value[1].s1 : 0.0;
        update_command(left_stick.s2, left_stick.s1, -right_x * vtheta_scale);
        return this->m_things;
    }

    if (yarp::os::Bottle* bot = thing.cast_as<yarp::os::Bottle>())
    {
        yAssert(validate_metaquest_bot(bot));

        const double left_x = bot->get(0).asFloat64();
        const double left_y = bot->get(1).asFloat64();
        const double right_x = bot->get(2).asFloat64();
        update_command(left_y, left_x, -right_x * vtheta_scale);
        return this->m_things;
    }

    yCError(JOY2VEL, "Unsupported message type received by joystickNwsYarp portmonitor");
    return thing;
}
