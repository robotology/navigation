/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef JOYSTICK2VELOCITYCOMMAND_H
#define JOYSTICK2VELOCITYCOMMAND_H

#include <yarp/os/MonitorObject.h>
#include <yarp/os/Things.h>
#include <yarp/dev/MobileBaseVelocity.h>

class Joy2vel : public yarp::os::MonitorObject
{
public:
    Joy2vel();
    virtual bool accept(yarp::os::Things& thing);
    virtual yarp::os::Things& update(yarp::os::Things& thing);
private:
    yarp::os::Things              m_things;
    yarp::dev::MobileBaseVelocity m_command;

    bool validate_bot(const yarp::os::Bottle* bot);
};

#endif
