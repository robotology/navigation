/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>

#ifndef NAV_VEL_INPUT_HANDLER_H
#define NAV_VEL_INPUT_HANDLER_H

class VelocityInputHandler :     public yarp::dev::Nav2D::INavigation2DVelocityActions,
                                 public yarp::dev::DeviceDriver
{
protected:
    std::string                                 m_localName;
    std::mutex                                  m_mutex;

    //internal type definition to store control output
    struct control_type
    {
        double linear_xvel = 0;
        double linear_yvel = 0;
        double angular_vel = 0;
        double timeout = 0;
        double reception_time =0;
    }
    m_control_out;
    double   m_max_timeout = 0.1;
    bool     timeout_printable = false;

public:
    VelocityInputHandler();

public:
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;

public:
    //INavigation2D methods
    bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;
    bool getLastVelocityCommand(double& x_vel, double& y_vel, double& theta_vel) override;
};

#endif
