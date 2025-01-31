/* 
 * Copyright (C)2018 ICub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <math.h>

#ifndef NAV_DEVICE_TEMPLATE_H
#define NAV_DEVICE_TEMPLATE_H

#define DEFAULT_THREAD_PERIOD 0.02 //s

class simpleVelocityNavigation : public yarp::os::PeriodicThread,
                                 public yarp::dev::Nav2D::INavigation2DVelocityActions,
                                 public yarp::dev::DeviceDriver
{
protected:
    std::string                                 m_localName;
    yarp::os::BufferedPort<yarp::os::Bottle>    m_port_commands_output;
    bool                                        m_send_zero_when_expired;
    //internal type definition to store control output
    struct control_type
    {
        double linear_xvel;
        double linear_yvel;
        double angular_vel;
        double timeout;
        double reception_time;
        control_type()
        {
            linear_xvel = 0; linear_yvel = 0; angular_vel = 0; timeout = 0; reception_time = 0;
        }
    }
    m_control_out;

public:
    simpleVelocityNavigation();

private:
    void send_command(control_type control_data);

public:
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    //INavigation2D methods
    yarp::dev::ReturnValue applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;
    yarp::dev::ReturnValue getLastVelocityCommand(double& x_vel, double& y_vel, double& theta_vel) override;
};

#endif
