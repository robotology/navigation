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
    yarp::dev::ReturnValue gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;
    yarp::dev::ReturnValue gotoTargetByRelativeLocation(double x, double y, double theta) override;
    yarp::dev::ReturnValue gotoTargetByRelativeLocation(double x, double y) override;
    yarp::dev::ReturnValue followPath(const yarp::dev::Nav2D::Map2DPath& path) override;
    yarp::dev::ReturnValue getAbsoluteLocationOfCurrentTarget(yarp::dev::Nav2D::Map2DLocation& target) override;
    yarp::dev::ReturnValue getRelativeLocationOfCurrentTarget(double& x, double& y, double& theta) override;
    yarp::dev::ReturnValue getNavigationStatus(yarp::dev::Nav2D::NavigationStatusEnum& status) override;
    yarp::dev::ReturnValue stopNavigation() override;
    yarp::dev::ReturnValue suspendNavigation(double time) override;
    yarp::dev::ReturnValue resumeNavigation() override;
    yarp::dev::ReturnValue getAllNavigationWaypoints(yarp::dev::Nav2D::TrajectoryTypeEnum trajectory_type, yarp::dev::Nav2D::Map2DPath& waypoints) override;
    yarp::dev::ReturnValue getCurrentNavigationWaypoint(yarp::dev::Nav2D::Map2DLocation& curr_waypoint) override;
    yarp::dev::ReturnValue getCurrentNavigationMap(yarp::dev::Nav2D::NavigationMapTypeEnum map_type, yarp::dev::Nav2D::MapGrid2D& map) override;
    yarp::dev::ReturnValue recomputeCurrentNavigationPath() override;
};
