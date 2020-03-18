/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "joystick2velocityCommand.h"

#include <yarp/os/Log.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>

Joy2vel::Joy2vel()
{
    this->things.setPortWriter(&this->command);
}

bool Joy2vel::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    if (bot == NULL) {
        yWarning("Joy2vel: expected type Bottle but got wrong data type!");
        return false;
    }
    if (bot->size() != 5) {
        yWarning("Joy2vel: expected Bottle of size 5 but got wrong size (%zd)!", bot->size());
        return false;
    }
    return true;
}

bool Joy2vel::validate_bot(const yarp::os::Bottle* bot)
{
    if (bot &&
        bot->size() == 5 &&
        bot->get(0).isInt() &&
        bot->get(1).isDouble() &&
        bot->get(2).isDouble() &&
        bot->get(3).isDouble() &&
        bot->get(0).isDouble())
    { 
        return true;
    }
    yError("Invalid bottle format");
    return false;
}

yarp::os::Things& Joy2vel::update(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    yAssert(bot);
    validate_bot(bot);

    if (bot->get(0).asInt()==3)
    {
        double percent = bot->get(4).asFloat64()/100.0;
        this->command.vel_x = bot->get(1).asFloat64() * percent;
        this->command.vel_y = bot->get(2).asFloat64() * percent;
        this->command.vel_theta = bot->get(3).asFloat64() * percent;
    }

    return this->things;
}
