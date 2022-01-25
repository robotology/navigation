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
#include <cmath>
#define _USE_MATH_DEFINES
//example
//yarp connect /joystickCtrl:o /baseControl /input /joystick:i tcp+recv.portmonitor+type.dll+file.joy2vel

YARP_LOG_COMPONENT(JOY2VEL, "navigation.Joy2Vel")

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

Joy2vel::Joy2vel()
{
    this->m_things.setPortWriter(&this->m_command);
}

bool Joy2vel::accept(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    if (bot == NULL) {
        yCWarning(JOY2VEL,"expected type Bottle but got wrong data type!");
        return false;
    }

    return validate_bot(bot);
}

bool Joy2vel::validate_bot(const yarp::os::Bottle* bot)
{
    if (!bot)
    {
        yCError(JOY2VEL, "Invalid bottle format: empty bottle");
        return false;
    }

    if (bot->size() < 5)
    {
        yCError(JOY2VEL, "Invalid bottle format: Size <5 or invalid data type");
        return false;
    }

    if ( isNumeric(bot->get(0))  == false ||
         bot->get(1).isFloat64() == false ||
         bot->get(2).isFloat64() == false ||
         bot->get(3).isFloat64() == false ||
         bot->get(4).isFloat64() == false )
    {
        yCError(JOY2VEL, "Invalid bottle format: invalid data type");
        return false;
    }

    //when the joystick button is not pressed, filter out the message.
    //This will eventually trigger a timeout on the receiver and
    //the control will be give to a different input source
    double percent = bot->get(4).asFloat64();
    if (percent < 10) {return false;}

    return true;
}

yarp::os::Things& Joy2vel::update(yarp::os::Things& thing)
{
    yarp::os::Bottle *bot = thing.cast_as<yarp::os::Bottle>();
    yAssert(bot);
    validate_bot(bot);

    if (bot->get(0).asInt32()==3)
    {
        double percent = bot->get(4).asFloat64()/100.0;
        this->m_command.vel_x = bot->get(1).asFloat64() * percent;
        this->m_command.vel_y = bot->get(2).asFloat64() * percent;
        this->m_command.vel_theta = bot->get(3).asFloat64() * percent;
    } else if (bot->get(0).asInt32()==2)
    {
        double percent = bot->get(4).asFloat64()/100.0;
        this->m_command.vel_x = bot->get(2).asFloat64() * percent  *  cos(bot->get(1).asFloat64()/ 180 * M_PI)  ;
        this->m_command.vel_y = bot->get(2).asFloat64() * percent * sin(bot->get(1).asFloat64() / 180 * M_PI);
        this->m_command.vel_theta = bot->get(3).asFloat64() * percent;
    } else
    {
        yCError(JOY2VEL, "Unsupported bottle format: Type!=3");
    }

    return this->m_things;
}
