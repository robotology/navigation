/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef INPUT_H
#define INPUT_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/MobileBaseVelocity.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IJoypadController.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/pids.h>
#include <yarp/dev/INavigation2D.h>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


class Input
{
private:
    Property            ctrl_options;
    string              localName;
    int                 thread_timeout_counter=0;

public:
    double              linear_vel_at_100_joy=0;
    double              angular_vel_at_100_joy=0;

protected:
    class inputManager
    {
        public:
        string                                            m_name;
        yarp::dev::PolyDriver*                            m_nws_dd=nullptr;
        yarp::dev::PolyDriver*                            m_inputmanager_dd = nullptr;
        yarp::dev::Nav2D::INavigation2DVelocityActions*   m_iVel=nullptr;

        int                                               m_timeout_counter = 0;
        int                                               data_received = 0;

        double                                            m_linear_speed = 0;
        double                                            m_angular_speed = 0;
        double                                            m_desired_direction = 0;
        double                                            m_pwm_gain = 0;
        inputManager(const inputManager&) = default;
        inputManager& operator=(const inputManager&) = default;
        inputManager() = default;
    };
    std::vector <inputManager>                    m_input;

public:

    /**
    * Default Constructor
    */
    Input();

    /**
    * Destructor
    */
    ~Input();

    /**
    * Opens the input module, parsing the given options
    * @param _options the configuration option for the module
    * @return true if the motor module opened successfully. False if a mandatory parameter is missing or invalid.
    */
    bool   open(Property &_options);

    /**
    * Closes the input module.
    */
    void   close();

    /**
    * Print some stats about the received input commands
    */
    void   printStats();
 
    /**
    * Receives the user input commands inputs. These source of these inputs may be a YARP port, a device (joypad), a ROS topic etc.
    * The robot commands are expressed as cartesian velocity in the robot reference frame.
    * @param linear_speed the mobile base linear speed
    * @param desired_direction the mobile base heading (linear_speed will be applied taking in account robot reference frame).
    * @param angular_speed the mobile base angular speed (in-place rotation).
    * @param pwm_gain the pwm gain (0-100). Joypad emergency button typically sets this value to zero to stop the robot. User modules, instead, do not use this value (always set to 100)/
    */
    void   read_inputs        (double& linear_speed, double& angular_speed, double& desired_direction, double& pwm_gain);
    
private:

    //Internal functions to extract velocity commands from a given bottle or from a joypad descriptor
    void   read_percent_polar (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_percent_cart  (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_polar   (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);
    void   read_speed_cart    (const Bottle *b, double& des_dir, double& lin_spd, double& ang_spd, double& pwm_gain);

    //Performs conversion from joypad stick units to metric units
    double get_linear_vel_at_100_joy()   { return linear_vel_at_100_joy; }
    double get_angular_vel_at_100_joy()  { return angular_vel_at_100_joy; }
};

#endif
