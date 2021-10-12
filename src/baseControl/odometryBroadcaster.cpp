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

#include "odometryBroadcaster.h"
#include "odometryHandler.h"
#include <yarp/os/LogStream.h>
#include <limits>

#define RAD2DEG 180.0/M_PI
#define DEG2RAD M_PI/180.0

YARP_LOG_COMPONENT(ODOM_BCST, "navigation.baseControl.OdometryBroadcaster")

OdometryBroadcaster::~OdometryBroadcaster()
{
    close();
}

OdometryBroadcaster::OdometryBroadcaster(OdometryHandler* _pp)
{
    m_dev=_pp;
}

bool OdometryBroadcaster::open(const Property &ctrl_options)
{
    if (ctrl_options.check("BASECTRL_GENERAL"))
    {
        yarp::os::Bottle g_group = ctrl_options.findGroup("BASECTRL_GENERAL");
        enable_ROS = (g_group.find("use_ROS").asBool() == true);
        if (enable_ROS) yCInfo(ODOM_BCST) << "ROS enabled";
        else
            yCInfo(ODOM_BCST) << "ROS not enabled";
    }
    else
    {
        yCError(ODOM_BCST) << "Missing [BASECTRL_GENERAL] section";
        return false;
    }

    localName = ctrl_options.find("local").asString();

    if (ctrl_options.check("ROS_ODOMETRY"))
    {
        yarp::os::Bottle ro_group = ctrl_options.findGroup("ROS_ODOMETRY");
        if (ro_group.check("odom_frame") == false) { yCError(ODOM_BCST) << "Missing odom_frame parameter"; return false; }
        if (ro_group.check("base_frame") == false) { yCError(ODOM_BCST) << "Missing base_frame parameter"; return false; }
        if (ro_group.check("topic_name") == false) { yCError(ODOM_BCST) << "Missing topic_name parameter"; return false; }
        odometry_frame_id = ro_group.find("odom_frame").asString();
        child_frame_id = ro_group.find("base_frame").asString();
        rosTopicName_odometry = ro_group.find("topic_name").asString();
    }
    else
    {
        yCError(ODOM_BCST) << "Missing [ROS_ODOMETRY] section";
        return false;
    }

    if (ctrl_options.check("ROS_FOOTPRINT"))
    {
        yarp::os::Bottle rf_group = ctrl_options.findGroup("ROS_FOOTPRINT");
        if (rf_group.check("topic_name") == false)  { yCError(ODOM_BCST) << "Missing topic_name parameter"; return false; }
        if (rf_group.check("footprint_diameter") == false)  { yCError(ODOM_BCST) << "Missing footprint_diameter parameter"; return false; }
        if (rf_group.check("footprint_frame") == false) { yCError(ODOM_BCST) << "Missing footprint_frame parameter"; return false; }
        footprint_frame_id = rf_group.find("footprint_frame").asString();
        rosTopicName_footprint = rf_group.find("topic_name").asString();
        footprint_diameter = rf_group.find("footprint_diameter").asFloat64();
    }
    else
    {
        yCError(ODOM_BCST) << "Missing [ROS_FOOTPRINT] section";
        return false;
    }

    if (enable_ROS)
    {

        if (!rosPublisherPort_odometry.topic(rosTopicName_odometry))
        {
            yCError(ODOM_BCST) << " opening " << rosTopicName_odometry << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        if (!rosPublisherPort_footprint.topic(rosTopicName_footprint))
        {
            yCError(ODOM_BCST) << " opening " << rosTopicName_footprint << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        if (!rosPublisherPort_tf.topic("/tf"))
        {
            yCError(ODOM_BCST) << " opening " << "/tf" << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        footprint.polygon.points.resize(12);
        double r = footprint_diameter;
        for (int i = 0; i< 12; i++)
        {
            double t = M_PI * 2 / 12 * i;
            footprint.polygon.points[i].x = (yarp::os::NetFloat32) (r*cos(t));
            footprint.polygon.points[i].y = (yarp::os::NetFloat32) (r*sin(t));
            footprint.polygon.points[i].z = 0;
        }
    }

    // open output ports
    bool ret = true;
    ret &= port_odometry.open((localName + "/odometry:o").c_str());
#if ODOMETER
    ret &= port_odometer.open((localName + "/odometer:o").c_str());
#endif
    ret &= port_vels.open((localName + "/velocity:o").c_str());
    if (ret == false)
    {
        yCError(ODOM_BCST) << "Unable to open module ports";
        return false;
    }

    return true;
}

void OdometryBroadcaster::close()
{
    port_odometry.interrupt();
    port_odometry.close();
    port_vels.interrupt();
    port_vels.close();
#if ODOMETER
    port_odometer.interrupt();
    port_odometer.close();
#endif

    if (enable_ROS)
    {
        rosPublisherPort_footprint.interrupt();
        rosPublisherPort_footprint.close();
        rosPublisherPort_odometry.interrupt();
        rosPublisherPort_odometry.close();
        rosPublisherPort_tf.interrupt();
        rosPublisherPort_tf.close();
    }
}

void OdometryBroadcaster::broadcast()
{
    if (m_dev)
    {
        if (m_dev->getOdometry(m_robot_odom))
        {
            yCError(ODOM_BCST) << "getOdometry() failed";
        }
    }

    m_timeStamp.update();
    if (port_odometry.getOutputCount()>0)
    {
        port_odometry.setEnvelope(m_timeStamp);
        yarp::dev::OdometryData &b = port_odometry.prepare();
        b.odom_x= m_robot_odom.odom_x; //position in the odom reference frame
        b.odom_y= m_robot_odom.odom_y;
        b.odom_theta= m_robot_odom.odom_theta;
        b.base_vel_x= m_robot_odom.base_vel_x; //velocity in the robot reference frame
        b.base_vel_y= m_robot_odom.base_vel_y;
        b.base_vel_theta= m_robot_odom.base_vel_theta;
        b.odom_vel_x= m_robot_odom.odom_vel_x; //velocity in the odom reference frame
        b.odom_vel_y= m_robot_odom.odom_vel_y;
        b.odom_vel_theta= m_robot_odom.odom_vel_theta;
        port_odometry.write();
    }

#if ODOMETER
    if (port_odometer.getOutputCount()>0)
    {
        port_odometer.setEnvelope(timeStamp);
        Bottle &t = port_odometer.prepare();
        t.clear();
        t.addFloat64(traveled_distance);
        t.addFloat64(traveled_angle);
        port_odometer.write();
    }
#endif

    if (port_vels.getOutputCount()>0)
    {
        port_vels.setEnvelope(m_timeStamp);
        Bottle &v = port_vels.prepare();
        v.clear();
        v.addFloat64(sqrt(m_robot_odom.odom_vel_x * m_robot_odom.odom_vel_x +
                          m_robot_odom.odom_vel_y * m_robot_odom.odom_vel_y));
        v.addFloat64(m_robot_odom.base_vel_theta);
        port_vels.write();
    }

    if (enable_ROS)
    {
        yarp::rosmsg::nav_msgs::Odometry &rosData = rosPublisherPort_odometry.prepare();
        rosData.header.seq = m_timeStamp.getCount();
        rosData.header.stamp = m_timeStamp.getTime();
        rosData.header.frame_id = odometry_frame_id;
        rosData.child_frame_id = child_frame_id;

        rosData.pose.pose.position.x = m_robot_odom.odom_x;
        rosData.pose.pose.position.y = m_robot_odom.odom_y;
        rosData.pose.pose.position.z = 0.0;
        yarp::rosmsg::geometry_msgs::Quaternion odom_quat;
        double halfYaw = m_robot_odom.odom_theta * DEG2RAD * 0.5;
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = sinYaw;
        odom_quat.w = cosYaw;
        rosData.pose.pose.orientation = odom_quat;
        rosData.twist.twist.linear.x = m_robot_odom.base_vel_x;
        rosData.twist.twist.linear.y = m_robot_odom.base_vel_y;
        rosData.twist.twist.linear.z = 0;
        rosData.twist.twist.angular.x = 0;
        rosData.twist.twist.angular.y = 0;
        rosData.twist.twist.angular.z = m_robot_odom.base_vel_theta * DEG2RAD;

        rosPublisherPort_odometry.write();
    }

    if (enable_ROS)
    {
        yarp::rosmsg::geometry_msgs::PolygonStamped &rosData = rosPublisherPort_footprint.prepare();
        rosData = footprint;
        rosData.header.seq = m_timeStamp.getCount();
        rosData.header.stamp = m_timeStamp.getTime();
        rosData.header.frame_id = footprint_frame_id;
        rosPublisherPort_footprint.write();
    }

    if (enable_ROS)
    {
        yarp::rosmsg::tf2_msgs::TFMessage &rosData = rosPublisherPort_tf.prepare();
        yarp::rosmsg::geometry_msgs::TransformStamped transform;
        transform.child_frame_id = child_frame_id;
        transform.header.frame_id = odometry_frame_id;
        transform.header.seq = m_timeStamp.getCount();
        transform.header.stamp = m_timeStamp.getTime();
        double halfYaw = m_robot_odom.odom_theta * DEG2RAD * 0.5;
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = sinYaw;
        transform.transform.rotation.w = cosYaw;
        transform.transform.translation.x = m_robot_odom.odom_x;
        transform.transform.translation.y = m_robot_odom.odom_y;
        transform.transform.translation.z = 0;
        if (rosData.transforms.size() == 0)
        {
            rosData.transforms.push_back(transform);
        }
        else
        {
            rosData.transforms[0] = transform;
        }
        rosPublisherPort_tf.write();
    }
}
