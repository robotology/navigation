/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/Map2DLocation.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <mutex>
#include <math.h>
#include <recovery_behaviors.h>

#ifndef NAVIGATION_STUCK_DETECTION_H
#define NAVIGATION_STUCK_DETECTION_H

class navigation_with_stuck_detection
{
    protected:
    bool                              m_stuck_recovery_enable = false;
    double                            m_stuck_linear_tolerance = 0.01;
    double                            m_stuck_angular_tolerance = 1.0;
    double                            m_stuck_time_tolerance = 30.0;
    std::string                       m_recovery_algorithm_name;
    generic_recovery_behaviour*       m_recovery_behavior = nullptr;
    double                            m_time_suspected_stuck;
    yarp::dev::Nav2D::Map2DLocation   m_last_position;

    virtual ~navigation_with_stuck_detection()
    {
        if (m_recovery_behavior != nullptr)
        {
            delete m_recovery_behavior;
            m_recovery_behavior = nullptr;
        }
    }
    
    bool initialize_recovery(yarp::os::Searchable& config);
    bool we_are_stuck(const yarp::dev::Nav2D::Map2DLocation& current_position);
};

#endif
