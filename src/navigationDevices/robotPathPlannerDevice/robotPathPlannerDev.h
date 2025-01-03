/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <recovery_behaviors.h>
#include <stuck_detection.h>
#include "pathPlannerCtrl.h"
#include <math.h>

class robotPathPlannerDev : public yarp::dev::DeviceDriver,
                            public yarp::dev::Nav2D::INavigation2DTargetActions,
                            public yarp::dev::Nav2D::INavigation2DControlActions,
                            public yarp::os::PortReader,
                            public navigation_with_stuck_detection
{
public:
    PlannerThread*                 m_plannerThread;
    yarp::os::Port                 m_rpcPort;
    std::string                    m_name = "/robotPathPlanner";

public:
    virtual bool open(yarp::os::Searchable& config) override;

    robotPathPlannerDev();

    //module cleanup
    virtual bool close() override;

    /* RPC responder */
    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual bool read(yarp::os::ConnectionReader& connection) override;

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
