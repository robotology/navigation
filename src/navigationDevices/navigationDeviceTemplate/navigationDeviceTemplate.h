/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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

class navigationDeviceTemplate : public yarp::os::PeriodicThread,
                                 public yarp::dev::Nav2D::INavigation2DTargetActions,
                                 public yarp::dev::Nav2D::INavigation2DControlActions,
                                 public yarp::dev::DeviceDriver
{
protected:
    yarp::dev::Nav2D::NavigationStatusEnum   m_navigation_status;

public:
    navigationDeviceTemplate();

public:
    virtual bool open(yarp::os::Searchable& config) override;
    virtual bool close() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    // INavigation2D methods
    yarp::dev::ReturnValue gotoTargetByAbsoluteLocation(yarp::dev::Nav2D::Map2DLocation loc) override;
    yarp::dev::ReturnValue gotoTargetByRelativeLocation(double x, double y) override;
    yarp::dev::ReturnValue gotoTargetByRelativeLocation(double x, double y, double theta) override;
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

#endif
