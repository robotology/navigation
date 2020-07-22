/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

/**
 * \section robotPathPlanner
 * This module performs global path-planning by generating a sequence of waypoints to be tracked by a local navigation algorithm.
 *  It receives a goal from the user either via RPC command or via yarp iNavigation2D interface and it computes a sequence of waypoints which are sent one by one to a local navigator module.
 * If the local navigation module fails to reach one of these waypoints, the global navigation is aborted too. 
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "pathPlannerCtrl.h"
#include <math.h>

class robotPathPlannerDev : public yarp::dev::DeviceDriver,
                            public yarp::dev::Nav2D::INavigation2DTargetActions,
                            public yarp::dev::Nav2D::INavigation2DControlActions,
                            public yarp::os::PortReader
{
public:
    PlannerThread*                 m_plannerThread;
    yarp::os::Port                 m_rpcPort;
    std::string                    m_local_name; 

public:
    virtual bool open(yarp::os::Searchable& config) override;

    robotPathPlannerDev();

    //module cleanup
    virtual bool close() override;

    /* RPC responder */
    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual bool read(yarp::os::ConnectionReader& connection) override;

public:
    /**
    * Sets a new navigation target, expressed in the absolute (map) coordinate frame.
    * @param loc the location to be reached
    * @return true/false if the command is accepted
    */
    bool gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;

    /**
    * //Sets a new relative target, expressed in local (robot) coordinate frame.
    * @param v a three-element vector (x,y,theta) representing the location to be reached
    * @return true/false if the command is accepted
    */
    bool gotoTargetByRelativeLocation(double x, double y, double theta) override;

    /**
    * //Sets a new relative target, expressed in local (robot) coordinate frame.
    * @param v a three-element vector (x,y,theta) representing the location to be reached
    * @return true/false if the command is accepted
    */
    bool gotoTargetByRelativeLocation(double x, double y) override;

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
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints) override;

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

    /**
    * Apply a velocity command. velocities are expressed in the robot reference frame
    * @param x [m/s]
    * @param y [m/s]
    * @param theta [deg/s]
    * @param timeout The velocity command expires after the specified amount of time (by default 0.1 seconds)
    * @return true/false
    */
    bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;
};
