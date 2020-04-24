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
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Subscriber.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/IMap2D.h>
#include <math.h>

#ifndef ISAAC_NAVIGATOR_H
#define ISAAC_NAVIGATOR_H

#define DEFAULT_THREAD_PERIOD 0.02 //s

class isaacNavigator : public yarp::dev::DeviceDriver,
    public yarp::os::PeriodicThread,
    public yarp::dev::Nav2D::INavigation2DTargetActions,
    public yarp::dev::Nav2D::INavigation2DControlActions
{
protected:
    bool m_map_dd_enable;
    bool m_localization_dd_enable;

    std::mutex                          m_mutex;
    yarp::dev::PolyDriver               m_pLoc;
    yarp::dev::Nav2D::ILocalization2D*  m_iLoc;
    yarp::dev::PolyDriver               m_pMap;
    yarp::dev::Nav2D::IMap2D*           m_iMap;

    yarp::dev::Nav2D::NavigationStatusEnum   m_navigation_status;
    std::string                       m_abs_frame_id;
    std::string                       m_local_name_prefix;
    std::string                       m_remote_localization;
    std::string                       m_remote_mapserver;
    std::string                       m_map_name;
    yarp::dev::Nav2D::Map2DLocation   m_current_position;
    yarp::dev::Nav2D::Map2DLocation   m_current_goal;
    yarp::dev::Nav2D::Map2DLocation   m_current_waypoint;
    yarp::dev::Nav2D::Map2DPath       m_global_plan;
    yarp::dev::Nav2D::Map2DPath       m_local_plan;

    double                            m_stats_time_curr;
    double                            m_stats_time_last;
    
    yarp::dev::Nav2D::MapGrid2D       m_local_map;
    yarp::dev::Nav2D::MapGrid2D       m_global_map;

    //ISAAC
    std::string m_port_navigation_status_name;
    std::string m_port_navigation_command_name;
    std::string m_port_global_trajectory_name;
    std::string m_port_local_trajectory_name;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_navigation_status;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_navigation_command;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_global_trajectory;
    yarp::os::BufferedPort<yarp::os::Bottle> m_port_local_trajectory;

public:
    isaacNavigator();
    ~isaacNavigator() {}

public:
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

private:
    std::string getStatusAsString(yarp::dev::Nav2D::NavigationStatusEnum status);

public:

    bool gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;
    bool gotoTargetByRelativeLocation(double x, double y, double theta) override;
    bool gotoTargetByRelativeLocation(double x, double y) override;
    bool getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target) override;
    bool getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta) override;
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status) override;
    bool stopNavigation() override;
    bool suspendNavigation(double time) override;
    bool resumeNavigation() override;
    bool applyVelocityCommand(double x_vel, double y_vel, double theta_vel, double timeout = 0.1) override;
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::Map2DPath& waypoints) override;
    bool getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint) override;
    bool getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map) override;
    bool recomputeCurrentNavigationPath() override;
};

#endif
