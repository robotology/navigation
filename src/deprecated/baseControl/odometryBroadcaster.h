/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef ODOMETRYBROADCASTER_H
#define ODOMETRYBROADCASTER_H

#include "odometryHandler.h"
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include <yarp/rosmsg/nav_msgs/Odometry.h>
#include <yarp/rosmsg/geometry_msgs/PolygonStamped.h>
#include <yarp/rosmsg/geometry_msgs/TransformStamped.h>
#include <yarp/rosmsg/tf2_msgs/TFMessage.h>
#include <yarp/dev/OdometryData.h>

#define _USE_MATH_DEFINES
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif 

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

class OdometryBroadcaster
{
protected:
    yarp::os::Stamp            m_timeStamp;
    yarp::dev::OdometryData    m_robot_odom;
    OdometryHandler*           m_dev=nullptr;

    //ROS
    bool                                            enable_ROS;
    yarp::os::Publisher<yarp::rosmsg::nav_msgs::Odometry>          rosPublisherPort_odometry;
    std::string                                     odometry_frame_id;
    std::string                                     child_frame_id;
    std::string                                     rosTopicName_odometry;

    yarp::os::Publisher<yarp::rosmsg::geometry_msgs::PolygonStamped>          rosPublisherPort_footprint;
    double                                                     footprint_diameter;
    std::string                                                rosTopicName_footprint;
    yarp::rosmsg::geometry_msgs::PolygonStamped                footprint;
    std::string                                                footprint_frame_id;

    yarp::os::Publisher<yarp::rosmsg::tf2_msgs::TFMessage>                    rosPublisherPort_tf;

protected:
    BufferedPort<yarp::dev::OdometryData>          port_odometry;
    BufferedPort<Bottle>                           port_odometer;
    BufferedPort<Bottle>                           port_vels;
    string                                         localName;

public:
    /**
    * Constructor
    * @param _driver is a pointer to a remoteControlBoard driver.
    */
    OdometryBroadcaster (OdometryHandler* _pp);

    /**
    * Default destructor
    */
    ~OdometryBroadcaster();
    
    /**
    * Initializes the odometry module.
    * @param options the options to be passed to odometry module.
    * @return true/false if the odometry module is opened successfully.
    */
    virtual bool   open(const Property &options);

    /**
    * Broadcast odometry data over YARP ports (or ROS topics)
    */
    virtual void   broadcast();

    /**
    * Terminates the execution of the odometry module
    */
    virtual void   close();

};

#endif
