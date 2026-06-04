/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#define _USE_MATH_DEFINES
#include <cmath>

#include "joystickServer2velocityCommand.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>

// Example:
// yarp connect /joystickCtrl:o /baseControl /input /joystick:i
//   tcp+recv.portmonitor+type.dll+file.joyServer2vel

YARP_LOG_COMPONENT(JOY2VEL, "navigation.Joy2Vel")

namespace
{
constexpr int metaquest_bottle_size = 4;
constexpr double metaquest_vtheta_scale = 30.0;
}

bool isNumeric(yarp::os::Value& val)
{
    if (val.isFloat32() ||
        val.isFloat64() ||
        val.isInt8() ||
        val.isInt16() ||
        val.isInt32() ||
        val.isInt64())
        {return true;}
    return false;
}

JoyServer2vel::JoyServer2vel()
{
    this->m_things.setPortWriter(&this->m_command);
}

bool JoyServer2vel::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    if (bot == NULL) {
        yCWarning(JOY2VEL,"expected type Bottle but got wrong data type!");
        return false;
    }

    return validate_bot(bot);
}

bool JoyServer2vel::validate_bot(const yarp::os::Bottle* bot)
{
    if (!bot)
    {
        yCError(JOY2VEL, "Invalid bottle format: empty bottle");
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

yarp::os::Things& JoyServer2vel::update(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    yAssert(bot);
    validate_bot(bot);

    const double left_x = bot->get(0).asFloat64();
    const double left_y = bot->get(1).asFloat64();
    const double right_x = bot->get(2).asFloat64();

    this->m_command.vel_x = left_y;
    this->m_command.vel_y = left_x;
    this->m_command.vel_theta = -right_x * metaquest_vtheta_scale;

    return this->m_things;
}
