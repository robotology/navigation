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


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IRangefinder2D.h>
#include <string>
#include <math.h>
#include <yarp/math/Math.h>
#include <yarp/math/Quaternion.h>
#include <algorithm>

#include "robotGoto.h"
#include "obstacles.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

std::string getStatusAsString(NavigationStatusEnum status)
{
    if      (status == navigation_status_idle)             return std::string("navigation_status_idle");
    else if (status == navigation_status_moving)           return std::string("navigation_status_moving");
    else if (status == navigation_status_waiting_obstacle) return std::string("navigation_status_waiting_obstacle");
    else if (status == navigation_status_goal_reached)     return std::string("navigation_status_goal_reached");
    else if (status == navigation_status_aborted)          return std::string("navigation_status_aborted");
    else if (status == navigation_status_paused)           return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move) return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking)         return std::string("navigation_thinking");
    yError("Unknown status of inner controller: '%d'!", status);
    return std::string("unknown");
}

GotoThread::GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
            RateThread(_period),
            m_rf(_rf),
            robotCtrl_options (options)
{
    yarp::os::Time::now();
    m_status = navigation_status_idle;
    m_obstacle_handler = 0;
    loc_timeout_counter             = TIMEOUT_MAX;
    odm_timeout_counter             = TIMEOUT_MAX;
    las_timeout_counter             = TIMEOUT_MAX;
    m_localization_data.resize(3, 0.0);
    m_retreat_counter = 0;
    m_enable_obstacles_emergency_stop = false;
    m_enable_obstacles_avoidance = false;
    m_enable_retreat = false;
    m_retreat_duration = 300;
    m_control_out.zero();
    pause_start                     = 0;
    pause_duration                  = 0;
    iLaser                          = 0;
    iTf                             = 0;
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    rosNode                         = 0;
}

bool GotoThread::rosInit(const yarp::os::Bottle& ros_group)
{
    if(ros_group.check("useGoalFromRosTopic") || ros_group.check("publishRosStuff"))
    {
        m_useGoalFromRosTopic  = ros_group.find("useGoalFromRosTopic").asBool();
        m_publishRosStuff      = ros_group.find("publishRosStuff").asBool();

        if(m_useGoalFromRosTopic || m_publishRosStuff)
        {
            if (ros_group.check("rosNodeName"))
            {
                rosNode = new yarp::os::Node( ros_group.find("rosNodeName").asString() );

                if (!m_useGoalFromRosTopic)
                {
                    yInfo() << "goal from ros topic deactivated";
                }
                else
                {

                    yInfo() << "activating ros goal input";

                    if (ros_group.check("goalTopicName"))
                    {
                        if (!rosGoalInputPort.topic( ros_group.find("goalTopicName").asString() ))
                        {
                            yError() << "error while opening goal subscriber";
                            return false;
                        }
                    }
                    else
                    {
                        yError() << "Initialization failed. Missing goalTopicName parameters in configuration file.";
                        return false;
                    }
                }

                if (!m_publishRosStuff)
                {
                    yInfo() << "Path and current goal publication deactivated";
                }
                else
                {

                    yInfo() << "activating current goal and path publication";

                    if (ros_group.check("currentGoalTopicName"))
                    {
                        if (!rosCurrentGoal.topic( ros_group.find("currentGoalTopicName").asString() ))
                        {
                            yError() << "error while opening current goal publisher";
                            return false;
                        }
                    }
                    else
                    {
                        yError() << "Initialization failed. Missing currentGoalTopicName parameters in configuration file. Current goal publication will be deactivated";
                        return false;
                    }

                    if (ros_group.check("localPlanTopicName"))
                    {
                        if (!localPlan.topic( ros_group.find("localPlanTopicName").asString() ))
                        {
                            yError() << "error while opening local plan publisher";
                            return false;
                        }
                    }
                    else
                    {
                        yError() << "Initialization failed. Missing localPlanTopicName parameters in configuration file. Path publication will be deactivated";
                        return false;
                    }

                    if (ros_group.check("globalPlanTopicName"))
                    {
                        if (!globalPlan.topic( ros_group.find("globalPlanTopicName").asString() ))
                        {
                            yError() << "error while opening global plan publisher";
                            return false;
                        }
                    }
                    else
                    {
                        yError() << "Initialization failed. Missing localPlanTopicName parameters in configuration file. Path publication will be deactivated";
                        return false;
                    }

                }

            }
            else
            {
                yInfo() << "Initialization failed. Missing rosNodeName parameters in configuration file.";
                return false;
            }

        }
    }
    return true;
}

bool GotoThread::threadInit()
{
    //read configuration parametes
    m_robot_is_holonomic = false;
    m_default_beta_angle_threshold  = m_beta_angle_threshold = 5;
    m_default_gain_ang = m_gain_ang = 0.05;
    m_default_gain_lin = m_gain_lin = 0.1;
    m_default_max_lin_speed         = m_max_lin_speed = 0.9;  //m/s
    m_default_max_ang_speed         = m_max_ang_speed = 10.0; //deg/s
    m_default_max_lin_speed         = m_min_lin_speed = 0.0;  //m/s
    m_default_max_ang_speed         = m_min_ang_speed = 0.0; //deg/s
    m_default_goal_tolerance_lin    = m_goal_tolerance_lin = 0.05;
    m_default_goal_tolerance_ang    = m_goal_tolerance_ang = 0.6;

    m_use_odometry               = true;
    m_use_localization_from_port = false;
    m_use_localization_from_tf   = false;
    m_useGoalFromRosTopic        = false;
    m_publishRosStuff            = false;
    m_obstacle_handler = new obstacles_class(m_rf);
    yInfo("Using following paramters: %s", m_rf.toString().c_str());

    Bottle ros_group = m_rf.findGroup("ROS");
    if (ros_group.isNull())
    {
        yInfo() << "Missing ROS group in configuration file. ros functionality will be deactivated";
    }
    else
    {
      if(!rosInit(ros_group))
      {
          return false;
      }
    }

    Bottle trajectory_group = m_rf.findGroup("ROBOT_TRAJECTORY");
    if (trajectory_group.isNull())
    {
        yError() << "Missing ROBOT_TRAJECTORY group!";
        return false;
    }

    if (trajectory_group.check("robot_is_holonomic")) { m_robot_is_holonomic = trajectory_group.find("robot_is_holonomic").asInt() == 1; }
    if (trajectory_group.check("max_beta_angle"))     { m_default_beta_angle_threshold = m_beta_angle_threshold = trajectory_group.find("max_beta_angle").asDouble(); }
    if (trajectory_group.check("ang_speed_gain"))     { m_default_gain_ang           = m_gain_ang           = trajectory_group.find("ang_speed_gain").asDouble(); }
    if (trajectory_group.check("lin_speed_gain"))     { m_default_gain_lin           = m_gain_lin           = trajectory_group.find("lin_speed_gain").asDouble(); }
    if (trajectory_group.check("max_lin_speed"))      { m_default_max_lin_speed      = m_max_lin_speed      = trajectory_group.find("max_lin_speed").asDouble(); }
    if (trajectory_group.check("max_ang_speed"))      { m_default_max_ang_speed      = m_max_ang_speed      = trajectory_group.find("max_ang_speed").asDouble(); }
    if (trajectory_group.check("min_lin_speed"))      { m_default_max_ang_speed      = m_min_lin_speed      = trajectory_group.find("min_lin_speed").asDouble(); }
    if (trajectory_group.check("min_ang_speed"))      { m_default_max_ang_speed      = m_min_ang_speed      = trajectory_group.find("min_ang_speed").asDouble(); }
    if (trajectory_group.check("goal_tolerance_lin")) { m_default_goal_tolerance_lin = m_goal_tolerance_lin = trajectory_group.find("goal_tolerance_lin").asDouble(); }
    if (trajectory_group.check("goal_tolerance_ang")) { m_default_goal_tolerance_lin = m_goal_tolerance_ang = trajectory_group.find("goal_tolerance_ang").asDouble(); }

    Bottle geometry_group = m_rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }
    Bottle localization_group = m_rf.findGroup("LOCALIZATION");
    if (localization_group.isNull())
    {
        yError() << "Missing LOCALIZATION group!";
        return false;
    }

    bool ff;
    ff  = geometry_group.check("robot_radius");
    ff &= geometry_group.check("laser_pos_x");
    ff &= geometry_group.check("laser_pos_y");
    ff &= geometry_group.check("laser_pos_theta");

    if (ff)
    {
        m_robot_radius = geometry_group.find("robot_radius").asDouble();
        m_robot_laser_x = geometry_group.find("laser_pos_x").asDouble();
        m_robot_laser_y = geometry_group.find("laser_pos_y").asDouble();
        m_robot_laser_t = geometry_group.find("laser_pos_theta").asDouble();
    }
    else
    {
        yError() << "Invalid/missing parameter in ROBOT_GEOMETRY group";
        return false;
    }

    if (localization_group.check("use_odometry"))               { m_use_odometry = (localization_group.find("use_odometry").asInt() == 1); }
    if (localization_group.check("use_localization_from_port")) { m_use_localization_from_port = (localization_group.find("use_localization_from_port").asInt() == 1); }
    if (localization_group.check("use_localization_from_tf"))   { m_use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }
    if (localization_group.check("robot_frame_id"))             { m_frame_robot_id = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id"))               { m_frame_map_id = localization_group.find("map_frame_id").asString(); }
    if (m_use_localization_from_port == true && m_use_localization_from_tf == true)
    {
        yError() << "`use_localization_from_tf` and `use_localization_from_port` cannot be true simulteneously!";
        return false;
    }

    Bottle btmp;
    btmp = m_rf.findGroup("OBSTACLES_AVOIDANCE");
    if (btmp.check("enable_obstacles_avoidance", Value(0)).asInt() == 1)
        m_enable_obstacles_avoidance = true;

    btmp = m_rf.findGroup("OBSTACLES_EMERGENCY_STOP");
    if (btmp.check("enable_obstacles_emergency_stop", Value(0)).asInt() == 1)
        m_enable_obstacles_emergency_stop = true;

    btmp = m_rf.findGroup("RETREAT_OPTION");
    if (btmp.check("enable_retreat", Value(0)).asInt() == 1)
        m_enable_retreat = true;

    m_retreat_duration = btmp.check("retreat_duration", Value(300)).asInt();

    //open module ports
    string localName = "/robotGoto";

    port_commands_output.open((localName + "/control:o").c_str());
    port_status_output.open((localName + "/status:o").c_str());
    port_odometry_input.open((localName + "/odometry:i").c_str());
    port_speak_output.open((localName + "/speak:o").c_str());
    port_gui_output.open((localName + "/gui:o").c_str());

    //localization
    if (m_use_localization_from_port)
    {
        port_localization_input.open((localName + "/localization:i").c_str());
    }

    if (m_use_localization_from_tf)
    {
        Property options;
        options.put("device", "transformClient");
        options.put("local", "/robotGoto/localizationTfClient");
        options.put("remote", "/transformServer");

        if (ptf.open(options) == false)
        {
            yError() << "Unable to open transform client";
            return false;
        }

        ptf.view(iTf);

        if (iTf == 0)
        {
            yError() << "Unable to view iTransform interface";
            return false;
        }
    }

    //open the laser interface
    Bottle laserBottle = m_rf.findGroup("LASER");

    if (laserBottle.isNull())
    {
        yError("LASER group not found,closing");
        return false;
    }

    if (laserBottle.check("laser_port") == false)
    {
        yError("laser_port param not found,closing");
        return false;
    }

    string laser_remote_port = laserBottle.find("laser_port").asString();

    Property options;
    options.put("device", "Rangefinder2DClient");
    options.put("local", "/robotGoto/laser:i");
    options.put("remote", laser_remote_port);
    options.put("period", 10);

    if (pLas.open(options) == false)
    {
        yError() << "Unable to open laser driver";
        return false;
    }

    pLas.view(iLaser);

    if (iLaser == 0)
    {
        yError() << "Unable to open laser interface";
        return false;
    }

    if (iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
    {
        yError() << "Unable to obtain laser scan limits";
        return false;
    }

    m_laser_angle_of_view = fabs(m_min_laser_angle) + fabs(m_max_laser_angle);

    //automatic connections for debug
    bool b = true;
    b = yarp::os::Network::connect("/robot/laser:o", "/yarpLaserScannerGui/laser:i");
    b = yarp::os::Network::connect("/robotGoto/gui:o", "/yarpLaserScannerGui/nav_display:i");

    //automatic port connections
    b = yarp::os::Network::connect("/baseControl/odometry:o", localName + "/odometry:i");
    b = yarp::os::Network::connect(localName + "/control:o", "/baseControl/control:i");

    /*
    b = Network::connect((localName+"/commands:o").c_str(),"/robot/control:i", "udp", false);
    if (!b) {yError ("Unable to connect the output command port!"); return false;}
    */

    return true;
}

void GotoThread::threadRelease()
{
    if (ptf.isValid()) ptf.close();
    if (pLas.isValid()) pLas.close();

    port_localization_input.interrupt();
    port_localization_input.close();
    
    port_target_input.interrupt();
    port_target_input.close();
    
    port_commands_output.interrupt();
    port_commands_output.close();
    
    port_status_output.interrupt();
    port_status_output.close();
    
    port_odometry_input.interrupt();
    port_odometry_input.close();
    
    port_speak_output.interrupt();
    port_speak_output.close();
    
    port_gui_output.interrupt();
    port_gui_output.close();

    rosGoalInputPort.interrupt();
    rosGoalInputPort.close();

    rosCurrentGoal.interrupt();
    rosCurrentGoal.close();

    localPlan.interrupt();
    localPlan.close();

    globalPlan.interrupt();
    globalPlan.close();

    if (m_obstacle_handler)
    {
        delete m_obstacle_handler;
        m_obstacle_handler = 0;
    }
}

void GotoThread::evaluateLocalization()
{
    //data is formatted as follows: x, y, angle (in degrees)
    yarp::sig::Vector *loc = 0;

    if (m_use_localization_from_port)
    {
        loc = port_localization_input.read(false);

        if (loc)
        {
            m_localization_data = *loc;
            loc_timeout_counter = 0;
        }
        else
        {
            loc_timeout_counter++;
            if (loc_timeout_counter>TIMEOUT_MAX) loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else if (m_use_localization_from_tf)
    {
        yarp::sig::Vector iv;
        yarp::sig::Vector pose;
        iv.resize(6, 0.0);
        pose.resize(6, 0.0);
        bool r = iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);

        if (r)
        {
            //data is formatted as follows: x, y, angle (in degrees)
            m_localization_data[X] = pose[0];
            m_localization_data[Y] = pose[1];
            m_localization_data[ANGLE] = pose[5] * RAD2DEG;
            loc_timeout_counter = 0;
        }
        else
        {
            loc_timeout_counter++;
            if (loc_timeout_counter > TIMEOUT_MAX) loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else
    {
        yWarning() << "Localization disabled";
    }

    if (loc_timeout_counter >= TIMEOUT_MAX)
    {
        if (m_status == navigation_status_moving)
        {
            yError("stopping navigation because of localization timeouts!");
            m_status = navigation_status_aborted;
        }
    }

    yarp::sig::Vector *odm = port_odometry_input.read(false);

    if (odm)
    {
        m_odometry_data = *odm;
        odm_timeout_counter = 0;
    }
    else
    {
        if (m_use_odometry)
            odm_timeout_counter++;

        if (odm_timeout_counter > TIMEOUT_MAX)
            odm_timeout_counter = TIMEOUT_MAX;
    }

    if (odm_timeout_counter >= TIMEOUT_MAX)
    {
        if (m_status == navigation_status_moving)
        {
            yError("stopping navigation because of odometry timeouts!");
            m_status = navigation_status_aborted;
        }
    }
}

void GotoThread::evaluateGoalFromTopic()
{
    geometry_msgs_PoseStamped* rosGoalData = rosGoalInputPort.read(false);

    if (rosGoalData != 0)
    {
        yInfo() << "received a goal from ros topic";
        yarp::sig::Vector v(3);
        yarp::math::Quaternion q;

        q.w() = rosGoalData->pose.orientation.w;
        q.x() = rosGoalData->pose.orientation.x;
        q.y() = rosGoalData->pose.orientation.y;
        q.z() = rosGoalData->pose.orientation.z;

        v[0] = rosGoalData->pose.position.x;
        v[1] = rosGoalData->pose.position.y;
        v[2] = -yarp::math::dcm2rpy(q.toRotationMatrix())[2] * RAD2DEG;

        setNewAbsTarget(v);
    }
}

void GotoThread::getLaserData()
{
    yarp::sig::Vector scan;
    bool ret = iLaser->getLaserMeasurement(m_laser_data);

    if (ret)
    {
        las_timeout_counter = 0;
    }
    else
    {
        las_timeout_counter++;
        if (las_timeout_counter>TIMEOUT_MAX) las_timeout_counter = TIMEOUT_MAX;
    }
}

void GotoThread::sendCurrentGoal()
{
    if (!m_publishRosStuff)
    {
        return;
    }
    geometry_msgs_PoseStamped& goal = rosCurrentGoal.prepare();
    static int        seq;
    yarp::math::Quaternion q;
    yarp::sig::Vector rpy(3);
    yarp::sig::Matrix m;

    rpy[0]                  = rpy[1] = 0;
    rpy[2]                  = m_target_data[ANGLE] * DEG2RAD;
    m                       = yarp::math::SE3inv(yarp::math::rpy2dcm(rpy)); 
    q.fromRotationMatrix(m);
    goal.header.frame_id    = m_frame_map_id;
    goal.header.seq         = seq;
    goal.pose.position.x    = m_target_data[X];
    goal.pose.position.y    = m_target_data[Y];
    goal.pose.position.z    = 0;
    goal.pose.orientation.w = q.w();
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();

    rosCurrentGoal.write();
    seq++;
}

void GotoThread::publishLocalPlan()
{
    if (iTf == 0)
    {
        yError() << "iTf not initialized";
        return;
    }

    if (m_status != navigation_status_moving || (m_control_out.linear_vel== 0 && m_control_out.angular_vel == 0))
    {
        return;
    }
    else
    {

        static int                seq;
        double                    radius, angle, distance;
        size_t                    pointCount;
        nav_msgs_Path&            path  = localPlan.prepare();
        yarp::sig::Matrix         map2robotMatrix;
        geometry_msgs_PoseStamped pose;
        yarp::sig::Vector         pos(4);

        //preparing header
        path.header.frame_id = m_frame_map_id;
        path.header.seq        = seq;
        path.header.stamp.sec  = int(yarp::os::Time::now());;
        path.header.stamp.nsec = (yarp::os::Time::now() - int(yarp::os::Time::now())) * 1000000;

        //drawing data
        pointCount = 10;
        radius     = 0.7;
        distance = sqrt(pow(m_target_data[X] - m_localization_data[X], 2) + pow(m_target_data[Y] - m_localization_data[Y], 2));
        angle = m_control_out.linear_dir * DEG2RAD;
        iTf->getTransform(m_frame_robot_id, m_frame_map_id, map2robotMatrix);
        path.poses.clear();
        path.poses.resize(pointCount + 1);

        for(size_t i = 0; i < pointCount + 1; i++)
        {
            pos[0]                  = m_control_out.linear_vel ? distance / pointCount * i : radius * cos(angle / pointCount * i);
            pos[1]                  = m_control_out.linear_vel ? 0 : radius * sin(angle / pointCount * i);
            pos[2]                  = 0.1;
            pos[3]                  = 1;
            pos                     = map2robotMatrix * pos;
            pose.header             = path.header;
            pose.pose.position.x    = pos[0];
            pose.pose.position.y    = pos[1];
            pose.pose.position.z    = pos[2];
            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            path.poses[i]           = pose;

        }

        localPlan.write();
        seq++;
    }

}

void GotoThread::getCurrentPos(yarp::sig::Vector& v)
{
    v.resize(m_localization_data.size());
    v = m_localization_data;
}

string GotoThread::getMapId()
{
    return m_frame_map_id;
}

double normalize_angle (double angle)
{
    //this function recevies an angle from (-inf,+inf) and returns an angle in (0,180) or (-180,0)
    angle = fmod(angle, 360);

    if (angle>180 && angle<360)
    {
        angle = angle - 360;
    }

    if (angle<-180 && angle>-360)
    {
        angle = angle + 360;
    }
    return angle;
}

void GotoThread::saturateRobotControls()
{
    if (m_min_ang_speed < 0){ yError() << "Invalid m_min_ang_speed value"; m_min_ang_speed = fabs(m_min_ang_speed); }
    if (m_max_ang_speed < 0){ yError() << "Invalid m_max_ang_speed value"; m_max_ang_speed = fabs(m_max_ang_speed); }
    if (m_min_lin_speed < 0){ yError() << "Invalid m_min_lin_speed value"; m_min_lin_speed = fabs(m_min_lin_speed); }
    if (m_min_lin_speed < 0){ yError() << "Invalid m_min_lin_speed value"; m_min_lin_speed = fabs(m_min_lin_speed); }

    //control saturation
    if (m_control_out.angular_vel>=0)
        m_control_out.angular_vel = std::max(m_min_ang_speed, std::min(m_control_out.angular_vel, m_max_ang_speed));
    else
        m_control_out.angular_vel = std::max(-m_max_ang_speed, std::min(m_control_out.angular_vel, -m_min_ang_speed));

    if (m_control_out.linear_vel>=0)
        m_control_out.linear_vel = std::max(m_min_lin_speed, std::min(m_control_out.linear_vel, m_max_lin_speed));
    else
        m_control_out.linear_vel = std::max(-m_max_lin_speed, std::min(m_control_out.linear_vel, -m_min_lin_speed));

    /*if (m_control_out.angular_vel > 0)
    {
        if (m_control_out.angular_vel  > +m_max_ang_speed) m_control_out.angular_vel = +m_max_ang_speed;
        if (m_control_out.angular_vel  < +m_min_ang_speed) m_control_out.angular_vel = +m_min_ang_speed;
    }
    else
    {
        if (m_control_out.angular_vel  < -m_max_ang_speed) m_control_out.angular_vel = -m_max_ang_speed;
        if (m_control_out.angular_vel  > -m_min_ang_speed) m_control_out.angular_vel = -m_min_ang_speed;
    }

    if (m_control_out.linear_vel > 0)
    {
        if (m_control_out.linear_vel > +m_max_lin_speed) m_control_out.linear_vel = +m_max_lin_speed;
        if (m_control_out.linear_vel < +m_min_lin_speed) m_control_out.linear_vel = +m_min_lin_speed;
    }
    else
    {
        if (m_control_out.linear_vel < -m_max_lin_speed) m_control_out.linear_vel = -m_max_lin_speed;
        if (m_control_out.linear_vel > -m_min_lin_speed) m_control_out.linear_vel = -m_min_lin_speed;
    }*/
}

void GotoThread::run()
{
    mutex.wait();
    if (m_useGoalFromRosTopic) evaluateGoalFromTopic();
    evaluateLocalization();
    getLaserData();

    //computes the control action
    m_control_out.zero();

    //gamma is the angle between the current robot heading and the target heading
    double unwrapped_localization_angle = (m_localization_data[ANGLE] < 0) ? m_localization_data[ANGLE] + 360 : m_localization_data[ANGLE];
    double unwrapped_target_angle = (m_target_data[ANGLE]       < 0) ? m_target_data[ANGLE] + 360 : m_target_data[ANGLE];
    double gamma                        = unwrapped_target_angle - unwrapped_localization_angle;
    gamma = normalize_angle(gamma);

    //beta is the angle between the current robot position and the target position IN THE WORLD REFERENCE FRAME
    double beta_world = atan2(m_target_data[Y] - m_localization_data[Y], m_target_data[X] - m_localization_data[X]) * RAD2DEG;
    //yDebug() << "beta world:" << beta_world;

    //distance is the distance between the current robot position and the target position
    double distance = sqrt(pow(m_target_data[X] - m_localization_data[X], 2) + pow(m_target_data[Y] - m_localization_data[Y], 2));

    //delta is the angle between the current robot position and the target position IN THE ROBOT REFERENCE FRAME
    double beta_robot = (beta_world - m_localization_data[ANGLE]);
    beta_robot = normalize_angle(beta_robot);
    //yDebug() << "beta robot:" << beta_robot;
    
    //check for obstacles, always performed
    bool obstacles_in_path = false;
    if (las_timeout_counter < 300)
    {
        obstacles_in_path = m_obstacle_handler->check_obstacles_in_path(m_laser_data);
        if (m_enable_obstacles_avoidance)  m_obstacle_handler->compute_obstacle_avoidance(m_laser_data);
    }

    double current_time = yarp::os::Time::now();
    double speed_ramp = (current_time - m_time_ob_obstacle_removal) / 2.0;

    //the finite state machine
    switch (m_status)
    {
        case navigation_status_preparing_before_move:
            if (m_retreat_counter > 0)
            {
                m_control_out.linear_dir = 180;
                m_control_out.linear_vel = 0.4;
                m_control_out.angular_vel = 0;
                m_retreat_counter--;
            }
            else
            {
                m_status = navigation_status_moving;
            }
        break;

        case navigation_status_moving:
            //Update the safety coefficient only if your are MOVING.
            //If you are WAITING_OBSTACLE, use the last valid safety_coeff until the
            //obstacle has been removed.
            m_obstacle_handler->set_safety_coeff(m_control_out.linear_vel / m_max_lin_speed);

            //compute the speed ramp after the removal of an obstacle
            speed_ramp      = (speed_ramp > 1.0) ? 1.0 : speed_ramp;
            m_control_out.linear_vel *= speed_ramp;
            m_control_out.angular_vel*= speed_ramp;

            //you are near to goal
            if (fabs(distance)< m_goal_tolerance_lin)
            {
                if (m_target_data.weak_angle)
                {
                    m_status = navigation_status_goal_reached;
                    yInfo("Goal reached!");
                }
                else
                {
                    //you are oriented as request
                    if (fabs(gamma) < m_goal_tolerance_ang)
                    {
                        m_status = navigation_status_goal_reached;
                        yInfo("Goal reached!");
                    }
                    //rotate until you are oriented as requested
                    else
                    {
                        //===========================
                        m_control_out.linear_vel = 0.0;
                        m_control_out.linear_dir = 0.0;
                        m_control_out.angular_vel = m_gain_ang * gamma;
                        //===========================
                    }
                }
            }
            else // you are far from the goal
            {
                //your heading is almost facing the goal, thus move forward
                if (fabs(beta_robot) < m_beta_angle_threshold)
                {
                    if (m_robot_is_holonomic)
                    {
                        //===========================
                        m_control_out.linear_vel = m_gain_ang * distance;
                        m_control_out.linear_dir = beta_robot;
                        m_control_out.angular_vel = m_gain_ang * beta_robot;
                        //===========================
                    }
                    else
                    {
                        //===========================
                        m_control_out.linear_vel = m_gain_ang * distance;
                        m_control_out.linear_dir = 0.0;
                        m_control_out.angular_vel = m_gain_ang * beta_robot;
                        //===========================
                    }
                }
                //do not move forward, just rotate unless you are almost facing the goal
                else
                {
                    //===========================
                    m_control_out.linear_vel = 0.0;
                    m_control_out.linear_dir = beta_robot;
                    m_control_out.angular_vel = m_gain_ang * beta_robot;
                    //===========================
                }
            }
              
            // check if you have to stop beacuse of an obstacle
            if (m_enable_obstacles_emergency_stop && obstacles_in_path)
            {
                yInfo ("Obstacles detected, stopping");
                Bottle b, tmp;

                m_status = navigation_status_waiting_obstacle;
                m_time_of_obstacle_detection = current_time;

                b.addString("Obstacles detected");
                tmp = port_speak_output.prepare();
                tmp.clear();
                tmp = b;
                port_speak_output.write();
            }
        break;

        case navigation_status_waiting_obstacle:
            if (!obstacles_in_path)
            {
                if (fabs(current_time - m_time_of_obstacle_detection) > 1.0)
                {

                    yInfo ("Obstacles removed, thank you");
                    Bottle b, tmp;
                    m_status = navigation_status_moving;

                    b.addString("Obstacles removed, thank you");
                    tmp = port_speak_output.prepare();
                    tmp.clear();
                    tmp = b;
                    port_speak_output.write();
                    m_time_ob_obstacle_removal = yarp::os::Time::now();
                }
            }
            else
            {
                if (fabs(current_time - m_time_of_obstacle_detection) > m_obstacle_handler->get_max_time_waiting_for_obstacle_removal())
                {
                    yError ("failed to recover from obstacle, goal aborted");
                    m_status = navigation_status_aborted;
                }
            }
        break;

        case navigation_status_paused:
            //check if pause is expired
            if (current_time - pause_start > pause_duration)
            {
                yInfo("pause expired! resuming");
                m_status = navigation_status_moving;
            }
        break;

        case navigation_status_idle:
        case navigation_status_thinking:
        case navigation_status_aborted:
        case navigation_status_goal_reached:
            //do nothing
        break;

        default:
            yError("unknown status:%d", m_status);
        break;
    }


    if (m_status == navigation_status_moving)
    {
        saturateRobotControls();
    }
    else
    {
        if (m_control_out.linear_dir != 0.0 || m_control_out.linear_vel != 0.0 || m_control_out.angular_vel != 0.0)
        {
            yWarning() << "RobotGoto computed a vel!=0 even if its status is not navigation_status_moving`";
            m_control_out.linear_dir = 0.0;    m_control_out.linear_vel = 0.0;    m_control_out.angular_vel = 0.0;
        }
    }

    //send commands
    if (m_publishRosStuff) publishLocalPlan();
    sendOutput();
    mutex.post();
}

void GotoThread::sendOutput()
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send the motors commands and the status to the yarp ports
    if (port_commands_output.getOutputCount() > 0)
    {
        Bottle &b = port_commands_output.prepare();
        port_commands_output.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar speed commands
        b.addDouble(m_control_out.linear_dir);    // angle in deg
        b.addDouble(m_control_out.linear_vel);    // lin_vel in m/s
        b.addDouble(m_control_out.angular_vel);    // ang_vel in deg/s
        b.addDouble(100);
        port_commands_output.write();
    }

    if (port_status_output.getOutputCount()>0)
    {
        string     string_out;
        string_out = getStatusAsString(m_status);
        Bottle &b  = port_status_output.prepare();

        port_status_output.setEnvelope(stamp);
        b.clear();
        b.addString(string_out.c_str());
        port_status_output.write();
    }

    if(port_gui_output.getOutputCount() > 0)
    {
        Bottle &b = port_gui_output.prepare();
        port_gui_output.setEnvelope(stamp);
        b.clear();
        b.addDouble(m_control_out.linear_dir);
        b.addDouble(m_control_out.linear_vel);
        b.addDouble(m_control_out.angular_vel);
        b.addDouble(m_obstacle_handler->m_angle_f);
        b.addDouble(m_obstacle_handler->m_angle_t);
        b.addDouble(m_obstacle_handler->m_w_f);
        b.addDouble(m_obstacle_handler->m_w_t);
        b.addDouble(m_obstacle_handler->m_max_obstacle_distance);
        b.addDouble(m_obstacle_handler->m_angle_g);
        port_gui_output.write();
    }
}

void GotoThread::setNewAbsTarget(yarp::sig::Vector target)
{
    //data is formatted as follows: x, y, angle
    m_target_data.weak_angle = false;
    if (target.size() == 2)
    {
        //if the angle information is missing use as final orientation the direction in which the robot has to move
        double beta = atan2(m_localization_data[1] - target[1], m_localization_data[0] - target[0])*180.0 / M_PI;
        double beta2 = beta-180;

        if (beta2>+180) beta2 = 360 - beta2;

        if (beta2<-180) beta2 = 360 + beta2;

        target.push_back(beta2);
        m_target_data.weak_angle = true;
    }
    m_target_data.target = target;
    m_status = navigation_status_preparing_before_move;
    if (m_enable_retreat)
    {
        m_retreat_counter = m_retreat_duration;
    }
    else
    {
        m_retreat_counter = 0;
    }

    yDebug("current pos: abs(%.3f %.3f %.2f)", m_localization_data[0], m_localization_data[1], m_localization_data[2]);
    yDebug("received new target: abs(%.3f %.3f %.2f)", m_target_data[0], m_target_data[1], m_target_data[2]);
    sendCurrentGoal();
}

Map2DLocation GotoThread::getCurrentAbsTarget()
{
    return Map2DLocation(m_frame_map_id, m_target_data[0], m_target_data[1], m_target_data[2]);
}

Map2DLocation GotoThread::getCurrentRelTarget()
{
    return Map2DLocation(m_frame_robot_id, m_target_data[0] - m_localization_data[0], m_target_data[1] - m_localization_data[1], m_target_data[2] - m_localization_data[2]);
}

void GotoThread::resetParamsToDefaultValue()
{
    m_beta_angle_threshold = m_default_beta_angle_threshold;
    m_gain_lin           = m_default_gain_lin;
    m_gain_ang           = m_default_gain_ang;
    m_goal_tolerance_lin = m_default_goal_tolerance_lin;
    m_goal_tolerance_ang = m_default_goal_tolerance_ang;
    m_max_lin_speed      = m_default_max_lin_speed;
    m_max_ang_speed      = m_default_max_ang_speed;
    m_min_lin_speed      = m_default_min_lin_speed;
    m_min_ang_speed      = m_default_min_ang_speed;
}

void GotoThread::setNewRelTarget(yarp::sig::Vector target)
{
    //target and localization data are formatted as follows: x, y, angle (in degrees)
    m_target_data.weak_angle = false;
    if (target.size() == 2)
    {
        target.push_back(0.0);
        m_target_data.weak_angle = true;
    }
    double a = m_localization_data[2] * DEG2RAD;
    //this is the inverse of the tranformation matrix from world to robot
    m_target_data[0] = +target[0] * cos(a) - target[1] * sin(a) + m_localization_data[0];
    m_target_data[1] = +target[0] * sin(a) + target[1] * cos(a) + m_localization_data[1];
    m_target_data[2] = target[2] + m_localization_data[2];
    m_status = navigation_status_preparing_before_move;
    if (m_enable_retreat)
    { 
        m_retreat_counter = m_retreat_duration;
    }
    else
    {
        m_retreat_counter = 0;
    }
    yInfo("received new target: abs(%.3f %.3f %.2f)", m_target_data[0], m_target_data[1], m_target_data[2]);
    sendCurrentGoal();
}

void GotoThread::pauseMovement(double secs)
{
    if (m_status == navigation_status_paused)
    {
        yWarning ( "already in pause!");
        return;
    }
    if (m_status != navigation_status_moving)
    {
        yWarning( "not moving!");
        return;
    }

    if (secs > 0)
    {
        yInfo ( "asked to pause for %f ", secs);
        pause_duration = secs;
    }
    else
    {
        yInfo( "asked to pause");
        pause_duration = 10000000;
    }
    m_status = navigation_status_paused;
    pause_start = yarp::os::Time::now();
}

void GotoThread::resumeMovement()
{
    yInfo( "asked to resume movement");
    m_status = navigation_status_moving;
}

void GotoThread::stopMovement()
{
    yInfo( "asked to stop");
    m_status = navigation_status_idle;
}

string GotoThread::getNavigationStatusAsString()
{
    return getStatusAsString(m_status);
}

int GotoThread::getNavigationStatusAsInt()
{
    return m_status;
}

void GotoThread::printStats()
{
    yDebug( "* robotGoto thread:");
    yDebug( "loc timeouts: %d", loc_timeout_counter);
    yDebug( "odm timeouts: %d", odm_timeout_counter);
    yDebug( "las timeouts: %d", las_timeout_counter);
    yDebug("status: %s", getStatusAsString(m_status).c_str());
}
