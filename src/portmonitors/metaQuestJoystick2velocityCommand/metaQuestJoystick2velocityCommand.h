/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef METAQUESTJOYSTICK2VELOCITYCOMMAND_H
#define METAQUESTJOYSTICK2VELOCITYCOMMAND_H

#include <yarp/dev/MobileBaseVelocity.h>
#include <yarp/os/MonitorObject.h>
#include <yarp/os/Things.h>

class MetaQuestJoy2vel : public yarp::os::MonitorObject
{
public:
    MetaQuestJoy2vel();
    bool accept(yarp::os::Things& thing) override;
    yarp::os::Things& update(yarp::os::Things& thing) override;

private:
    bool validate_bot(const yarp::os::Bottle* bot);

    yarp::os::Things m_things;
    yarp::dev::MobileBaseVelocity m_command;
};

#endif
