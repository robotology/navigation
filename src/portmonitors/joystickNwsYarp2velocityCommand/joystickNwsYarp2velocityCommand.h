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

namespace yarp::dev
{
class AllJoyData;
class StickDataList;
}

namespace yarp::os
{
class Bottle;
}

class JoyNwsYarp2vel : public yarp::os::MonitorObject
{
public:
    JoyNwsYarp2vel();
    virtual bool accept(yarp::os::Things& thing);
    virtual yarp::os::Things& update(yarp::os::Things& thing);
private:
    yarp::os::Things              m_things;
    yarp::dev::MobileBaseVelocity m_command;

    bool validate_all_joy_data(const yarp::dev::AllJoyData* data);
    bool validate_stick_data_list(const yarp::dev::StickDataList* data);
    bool validate_metaquest_bot(const yarp::os::Bottle* bot);
    void update_command(double x, double y, double theta);
};

#endif
