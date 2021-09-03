/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "joystick2velocityCommand.h"

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>

YARP_LOG_COMPONENT(JOY2VEL, "navigation.Joy2Vel")

Joy2vel::Joy2vel()
{
    this->things.setPortWriter(&this->command);
}

bool Joy2vel::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    if (bot == NULL) {
        yCWarning(JOY2VEL,"expected type Bottle but got wrong data type!");
        return false;
    }
    if (bot->size() != 5) {
        yCWarning(JOY2VEL,"expected Bottle of size 5 but got wrong size (%zd)!", bot->size());
        return false;
    }
    return true;
}

bool Joy2vel::validate_bot(const yarp::os::Bottle* bot)
{
    if (bot &&
        bot->size() == 5 &&
        bot->get(0).isInt32() &&
        bot->get(1).isFloat64() &&
        bot->get(2).isFloat64() &&
        bot->get(3).isFloat64() &&
        bot->get(0).isFloat64())
    { 
        return true;
    }
    yCError(JOY2VEL,"Invalid bottle format");
    return false;
}

yarp::os::Things& Joy2vel::update(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    yAssert(bot);
    validate_bot(bot);

    if (bot->get(0).asInt32()==3)
    {
        double percent = bot->get(4).asFloat64()/100.0;
        this->command.vel_x = bot->get(1).asFloat64() * percent;
        this->command.vel_y = bot->get(2).asFloat64() * percent;
        this->command.vel_theta = bot->get(3).asFloat64() * percent;
    }

    return this->things;
}
