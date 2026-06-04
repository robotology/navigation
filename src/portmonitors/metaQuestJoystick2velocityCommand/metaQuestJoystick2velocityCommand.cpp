/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "metaQuestJoystick2velocityCommand.h"

#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace
{
constexpr std::size_t bottle_size = 4;

// Meta Quest stick layout currently assumed to be:
// [left_x, left_y, right_x, right_y]
constexpr int left_x_index = 0;
constexpr int left_y_index = 1;
constexpr int right_x_index = 2;

constexpr double vtheta_scale = 30.0;
}

YARP_LOG_COMPONENT(METAQUEST_JOY2VEL, "navigation.MetaQuestJoy2Vel")

bool isNumeric(yarp::os::Value& val)
{
    if (val.isFloat32() ||
        val.isFloat64() ||
        val.isInt8() ||
        val.isInt16() ||
        val.isInt32() ||
        val.isInt64()) {
        return true;
    }
    return false;
}

MetaQuestJoy2vel::MetaQuestJoy2vel()
{
    m_things.setPortWriter(&m_command);
}

bool MetaQuestJoy2vel::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle* bot = thing.cast_as<yarp::os::Bottle>();
    if (bot == nullptr) {
        yCWarning(METAQUEST_JOY2VEL, "expected type Bottle but got wrong data type!");
        return false;
    }

    return validate_bot(bot);
}

bool MetaQuestJoy2vel::validate_bot(const yarp::os::Bottle* bot)
{
    if (bot == nullptr) {
        yCError(METAQUEST_JOY2VEL, "Invalid bottle format: empty bottle");
        return false;
    }

    if (bot->size() < static_cast<int>(bottle_size)) {
        yCError(METAQUEST_JOY2VEL, "Invalid bottle format: size < 4");
        return false;
    }

    for (int i = 0; i < static_cast<int>(bottle_size); ++i) {
        yarp::os::Value value = bot->get(i);
        if (!isNumeric(value)) {
            yCError(METAQUEST_JOY2VEL, "Invalid bottle format: item %d is not numeric", i);
            return false;
        }
    }

    return true;
}

yarp::os::Things& MetaQuestJoy2vel::update(yarp::os::Things& thing)
{
    yarp::os::Bottle* bot = thing.cast_as<yarp::os::Bottle>();
    yAssert(bot);
    validate_bot(bot);

    const double left_x = bot->get(left_x_index).asFloat64();
    const double left_y = bot->get(left_y_index).asFloat64();
    const double right_x = bot->get(right_x_index).asFloat64();

    m_command.vel_x = left_y;
    m_command.vel_y = left_x;
    m_command.vel_theta = -right_x * vtheta_scale;

    return m_things;
}
