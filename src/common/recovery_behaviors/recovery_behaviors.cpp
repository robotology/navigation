/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#define _USE_MATH_DEFINES
#include "recovery_behaviors.h"
#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef RAD2DEG
#define RAD2DEG 180/M_PI
#endif

#ifndef DEG2RAD
#define DEG2RAD M_PI/180
#endif


//////////////////////////
recovery_behavior::recovery_behavior(BufferedPort<yarp::os::Bottle>* command_port,
    IRangefinder2D* iLas ,
    ILocalization2D* m_iLoc,
    IMap2D* m_iMap) : PeriodicThread (0.010)
{
    m_port_commands_output = command_port;
}

void recovery_behavior::run()
{
    double current_time = yarp::os::Time::now();
    Bottle& b = m_port_commands_output->prepare();
    //m_port_commands_output.setEnvelope(stamp);
    b.clear();
    b.addInt(2);               // polar speed commands
    b.addDouble(0.0);          // angle in deg
    b.addDouble(0.0);          // lin_vel in m/s
    b.addDouble(ang_speed);    // ang_vel in deg/s
    b.addDouble(100);
    m_port_commands_output->write();

    if (current_time-start_time >10.0)
    {
         status_completed=true;
    }
}

bool recovery_behavior::is_status_completed()
{
    return status_completed;
}

bool recovery_behavior::start_recovery_behavior()
{
    this->start();
    return true;
}

bool recovery_behavior::stop_recovery_behavior()
{
    this->askToStop();
    return true;
}

bool recovery_behavior::threadInit()
{
    start_time =yarp::os::Time::now();
    double ang_speed= 10.0;
    //this->setPeriod(0.010);
    status_completed=false;
    return true;
}

void recovery_behavior::threadRelease()
{
}
