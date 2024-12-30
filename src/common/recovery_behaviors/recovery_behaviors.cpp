/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
    b.addInt32(2);               // polar speed commands
    b.addFloat64(0.0);          // angle in deg
    b.addFloat64(0.0);          // lin_vel in m/s
    b.addFloat64(ang_speed);    // ang_vel in deg/s
    b.addFloat64(100);
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
