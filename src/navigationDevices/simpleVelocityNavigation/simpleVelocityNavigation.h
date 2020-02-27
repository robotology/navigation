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
                                 public yarp::dev::Nav2D::INavigation2DTargetActions,
                                 public yarp::dev::Nav2D::INavigation2DControlActions,
                                 public yarp::dev::DeviceDriver
{
protected:
    std::string                                 m_localName;
    yarp::dev::Nav2D::NavigationStatusEnum      m_navigation_status;
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
    /**
    * Sets a new navigation target, expressed in the absolute (map) coordinate frame.
    * @param loc the location to be reached
    * @return true/false if the command is accepted
    */
    bool gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;

    /**
    * Ask the robot to reach a position defined in the robot reference frame. The final orientation of the goal is unspecified.
    * @param x
    * @param y
    * @return true/false
    */
    bool gotoTargetByRelativeLocation(double x, double y) override;

    /**
    * //Sets a new relative target, expressed in local (robot) coordinate frame.
    * @param v a three-element vector (x,y,theta) representing the location to be reached
    * @return true/false if the command is accepted
    */
    bool gotoTargetByRelativeLocation(double x, double y, double theta) override;

    /**
    * Apply a velocity command. velocities are expressed in the robot reference frame
    * @param x [m/s]
    * @param y [m/s]
    * @param theta [deg/s]
    * @param timeout The velocity command expires after the specified amount of time (by default 0.1 seconds)
    * @return true/false
    */
    virtual bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;

    /**
    * //Gets the last target set through a setNewAbsTarget() command.
    * @return a Map2DLocation containing data of the current target.
    * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
    */
    bool getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target) override;

    /**
    * //Gets the last target set through a setNewRelTarget command, expressed in absolute coordinates.
    * @param a Map2DLocation containing data of the current target.
    * @return true if a target is currently available, false otherwise (in this case returned target is invalid)
    */
    bool getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta) override;

    /**
    * //Gets the status of the current navigation task. Typically stored into navigation_status variable.
    * @return the current navigation status expressed as NavigationStatusEnum.
    */
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status) override;

    /**
    * //Stops the current navigation task.
    * @return true/false if the command is executed successfully.
    */
    bool stopNavigation() override;

    /**
    * //Pauses the current navigation task.
    * @return true/false if the command is executed successfully.
    */
    bool suspendNavigation(double time) override;

    /**
    * //Resumes a previously paused navigation task.
    * @return true/false if the command is executed successfully.
    */
    bool resumeNavigation() override;

    /**
    * Returns the list of waypoints generated by the navigation algorithm
    * @param waypoints the list of waypoints generated by the navigation algorithm
    * @return true/false
    */
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::Map2DPath& waypoints) override;

    /**
    * Returns the current waypoint pursued by the navigation algorithm
    * @param curr_waypoint the current waypoint pursued by the navigation algorithm
    * @return true/false
    */
    bool getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint) override;

    /**
    * Returns the current navigation map processed by the navigation algorithm
    * @param map_type the map to be requested (e.g. global, local, etc.)
    * @param map the map, currently used by the navigation algorithm
    * @return true/false
    */
    bool getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map) override;

    /**
    * Forces the navigation system to recompute the path from the current robot position to the current goal.
    * If no goal has been set, the command has no effect.
    * @return true/false
    */
    bool recomputeCurrentNavigationPath() override;
};

#endif
