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

#include "odometry.h"
#include <yarp/os/LogStream.h>
#include <limits>

#define RAD2DEG 180.0/3.14159
#define DEG2RAD 3.14159/180.0

inline TickTime normalizeSecNSec(double yarpTimeStamp)
{
    uint64_t time = (uint64_t)(yarpTimeStamp * 1000000000UL);
    uint64_t nsec_part = (time % 1000000000UL);
    uint64_t sec_part = (time / 1000000000UL);
    TickTime ret;

    if (sec_part > std::numeric_limits<unsigned int>::max())
    {
        yWarning() << "Timestamp exceeded the 64 bit representation, resetting it to 0";
        sec_part = 0;
    }

    ret.sec = (yarp::os::NetUint32) sec_part;
    ret.nsec = (yarp::os::NetUint32) nsec_part;
    return ret;
}

Odometry::~Odometry()
{
    close();
}

Odometry::Odometry(unsigned int _period, PolyDriver* _driver)
{
    period = _period;
    control_board_driver= _driver;
    odom_x=0;
    odom_y=0;
    odom_z=0;
    odom_theta=0;

    odom_vel_x=0;
    odom_vel_y=0;
    odom_vel_lin=0;
    odom_vel_theta=0;
    base_vel_x = 0;
    base_vel_y = 0;
    base_vel_lin = 0;
    base_vel_theta = 0;
    traveled_distance=0;
    traveled_angle=0;
    rosNode = NULL;
    rosMsgCounter=0;
}

bool Odometry::open(ResourceFinder &_rf, Property &options)
{
    if (ctrl_options.check("GENERAL"))
    {
        yarp::os::Bottle g_group = ctrl_options.findGroup("GENERAL");
        enable_ROS = (g_group.find("use_ROS").asInt() == 1);
        if (enable_ROS) yInfo() << "ROS enabled";
        else
            yInfo() << "ROS not enabled";
    }
    else
    {
        yError() << "Missing [GENERAL] section";
        return false;
    }

    if (ctrl_options.check("ROS_GENERAL"))
    {
        yarp::os::Bottle rg_group = ctrl_options.findGroup("ROS_GENERAL");
        if (rg_group.check("node_name") == false)  { yError() << "Missing node_name parameter"; return false; }
        rosNodeName = rg_group.find("node_name").asString();
    }
    else
    {
        yError() << "Missing [ROS_GENERAL] section";
        return false;
    }

    if (ctrl_options.check("ROS_ODOMETRY"))
    {
        yarp::os::Bottle ro_group = ctrl_options.findGroup("ROS_ODOMETRY");
        if (ro_group.check("odom_frame") == false) { yError() << "Missing odom_frame parameter"; return false; }
        if (ro_group.check("base_frame") == false) { yError() << "Missing base_frame parameter"; return false; }
        if (ro_group.check("topic_name") == false) { yError() << "Missing topic_name parameter"; return false; }
        odometry_frame_id = ro_group.find("odom_frame").asString();
        child_frame_id = ro_group.find("base_frame").asString();
        rosTopicName_odometry = ro_group.find("topic_name").asString();
    }
    else
    {
        yError() << "Missing [ROS_ODOMETRY] section";
        return false;
    }

    if (ctrl_options.check("ROS_FOOTPRINT"))
    {
        yarp::os::Bottle rf_group = ctrl_options.findGroup("ROS_FOOTPRINT");
        if (rf_group.check("topic_name") == false)  { yError() << "Missing topic_name parameter"; return false; }
        if (rf_group.check("footprint_diameter") == false)  { yError() << "Missing footprint_diameter parameter"; return false; }
        if (rf_group.check("footprint_frame") == false) { yError() << "Missing footprint_frame parameter"; return false; }
        footprint_frame_id = rf_group.find("footprint_frame").asString();
        rosTopicName_footprint = rf_group.find("topic_name").asString();
        footprint_diameter = rf_group.find("footprint_diameter").asDouble();
    }
    else
    {
        yError() << "Missing [ROS_FOOTPRINT] section";
        return false;
    }

    if (enable_ROS)
    {
        rosNode = new yarp::os::Node(rosNodeName);   // add a ROS node
        if (rosNode == NULL)
        {
            yError() << " opening " << rosNodeName << " Node, check your yarp-ROS network configuration\n";
            return false;
        }

        if (!rosPublisherPort_odometry.topic(rosTopicName_odometry))
        {
            yError() << " opening " << rosTopicName_odometry << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        if (!rosPublisherPort_footprint.topic(rosTopicName_footprint))
        {
            yError() << " opening " << rosTopicName_footprint << " Topic, check your yarp-ROS network configuration\n";
            return false;
        }

        if (!rosPublisherPort_tf.topic("/tf"))
        {
            yError() << " opening " << "/tf" << " Topic, check your yarp-ROS network configuration\n";
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
    return true;
}

void Odometry::close()
{
    port_odometry.interrupt();
    port_odometry.close();
    port_odometer.interrupt();
    port_odometer.close();
    port_vels.interrupt();
    port_vels.close();
}

void Odometry::broadcast()
{
    mutex.wait();
    timeStamp.update();
    if (port_odometry.getOutputCount()>0)
    {
        port_odometry.setEnvelope(timeStamp);
        Bottle &b = port_odometry.prepare();
        b.clear();
        b.addDouble(odom_x);
        b.addDouble(odom_y);
        b.addDouble(odom_theta);
        b.addDouble(odom_vel_x);
        b.addDouble(odom_vel_y);
        b.addDouble(odom_vel_theta);
        port_odometry.write();
    }

    if (port_odometer.getOutputCount()>0)
    {
        port_odometer.setEnvelope(timeStamp);
        Bottle &t = port_odometer.prepare();
        t.clear();
        t.addDouble(traveled_distance);
        t.addDouble(traveled_angle);
        port_odometer.write();
    }

    if (port_vels.getOutputCount()>0)
    {
        port_vels.setEnvelope(timeStamp);
        Bottle &v = port_vels.prepare();
        v.clear();
        v.addDouble(base_vel_lin);
        v.addDouble(base_vel_theta);
        port_vels.write();
    }

    if (enable_ROS)
    {
        nav_msgs_Odometry &rosData = rosPublisherPort_odometry.prepare();
        rosData.header.seq = rosMsgCounter;
        rosData.header.stamp = normalizeSecNSec(yarp::os::Time::now());
        rosData.header.frame_id = odometry_frame_id;
        rosData.child_frame_id = child_frame_id;

        rosData.pose.pose.position.x = odom_x;
        rosData.pose.pose.position.y = odom_y;
        rosData.pose.pose.position.z = 0.0;
        geometry_msgs_Quaternion odom_quat;
        double halfYaw = odom_theta / 180.0*M_PI * 0.5;
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = sinYaw;
        odom_quat.w = cosYaw;
        rosData.pose.pose.orientation = odom_quat;
        rosData.twist.twist.linear.x = odom_vel_x;
        rosData.twist.twist.linear.y = odom_vel_y;
        rosData.twist.twist.linear.z = 0;
        rosData.twist.twist.angular.x = 0;
        rosData.twist.twist.angular.y = 0;
        rosData.twist.twist.angular.z = odom_vel_theta / 180.0*M_PI;

        rosPublisherPort_odometry.write();
    }

    if (enable_ROS)
    {
        geometry_msgs_PolygonStamped &rosData = rosPublisherPort_footprint.prepare();
        rosData = footprint;
        rosData.header.seq = rosMsgCounter;
        rosData.header.stamp = normalizeSecNSec(yarp::os::Time::now());
        rosData.header.frame_id = footprint_frame_id;
        rosPublisherPort_footprint.write();
    }

    if (enable_ROS)
    {
        tf_tfMessage &rosData = rosPublisherPort_tf.prepare();
        geometry_msgs_TransformStamped transform;
        transform.child_frame_id = child_frame_id;
        transform.header.frame_id = odometry_frame_id;
        transform.header.seq = rosMsgCounter;
        transform.header.stamp = normalizeSecNSec(yarp::os::Time::now());
        double halfYaw = odom_theta / 180.0*M_PI * 0.5;
        double cosYaw = cos(halfYaw);
        double sinYaw = sin(halfYaw);
        transform.transform.rotation.x = 0;
        transform.transform.rotation.y = 0;
        transform.transform.rotation.z = sinYaw;
        transform.transform.rotation.w = cosYaw;
        transform.transform.translation.x = odom_x;
        transform.transform.translation.y = odom_y;
        transform.transform.translation.z = odom_z;
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

    rosMsgCounter++;

    mutex.post();
}

double Odometry::get_base_vel_lin()
{
    return this->base_vel_lin;
}

double Odometry::get_base_vel_theta()
{
    return this->base_vel_theta;
}
