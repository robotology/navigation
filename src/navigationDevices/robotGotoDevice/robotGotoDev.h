/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * \section robotGoto
 * robotGoto is a local navigation module. It receives a position cartesian target, either absolute (respect to the map origin) or relative (respect to the robot frame) and computes 
 * the cartesian velocity commands to be sent to baseControl module.
 * The module can be configured to stop the navigation if an obstacle is detected, or perform obstacle avoidance using an
 * APF-based approach.
 * A detailed description of configuration parameters available for the module is provided in the README.md file.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>
#include "robotGotoCtrl.h"
#include "navigation_defines.h"

class robotGotoDev;

class robotGotoRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    robotGotoDev * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response);

public:
    robotGotoRPCHandler() : interface(NULL) { }
    void setInterface(robotGotoDev* iface);
};

class robotGotoDev : public yarp::dev::DeviceDriver,
                     public yarp::dev::Nav2D::INavigation2DTargetActions,
                     public yarp::dev::Nav2D::INavigation2DControlActions
{
public:
    GotoThread          *gotoThread;
    robotGotoRPCHandler rpcPortHandler;
    yarp::os::Port      rpcPort;
    std::string         m_name; 

public:
    virtual bool open(yarp::os::Searchable& config) override;

    robotGotoDev();

    //module cleanup
    virtual bool close() override;

    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

public:
    // INavigation2D methods
    bool gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;
    bool gotoTargetByRelativeLocation(double x, double y, double theta) override;
    bool gotoTargetByRelativeLocation(double x, double y) override;
    bool followPath(const yarp::dev::Nav2D::Map2DPath& path) override;
    bool getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target) override;
    bool getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta) override;
    bool getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status) override;
    bool stopNavigation() override;
    bool suspendNavigation(double time) override;
    bool resumeNavigation() override;
    bool getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints) override;
    bool getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint) override;
    bool getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map) override;
    bool recomputeCurrentNavigationPath() override;
};
