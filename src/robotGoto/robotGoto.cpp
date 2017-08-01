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
    else if (status == navigation_status_failing)          return std::string("navigation_status_failing");
    else if (status == navigation_status_paused)           return std::string("navigation_status_paused");
    else if (status == navigation_status_preparing_before_move) return std::string("navigation_status_preparing_before_move");
    else if (status == navigation_status_thinking)         return std::string("navigation_thinking");
    yError("Unknown status of inner controller: '%d'!", status);
    return std::string("unknown");
}

GotoThread::GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
            RateThread(_period),
            m_rf(_rf),
            m_robotCtrl_options(options)
{
    yarp::os::Time::now();
    m_status = navigation_status_idle;
    m_obstacle_handler = 0;
    m_loc_timeout_counter = TIMEOUT_MAX;
    m_las_timeout_counter = TIMEOUT_MAX;
    m_retreat_starting_time = 0;
    m_retreat_duration_time = 0;
    m_enable_obstacles_emergency_stop = false;
    m_enable_obstacles_avoidance = false;
    m_enable_retreat = false;
    m_retreat_duration_default = 0.3;
    m_control_out.zero();
    m_pause_start = 0;
    m_pause_duration = 0;
    m_iLaser = 0;
    m_iLoc = 0;
    m_min_laser_angle = 0;
    m_max_laser_angle = 0;
    m_robot_radius = 0;
    m_robot_laser_x = 0;
    m_robot_laser_y = 0;
    m_robot_laser_t = 0;
    m_rosNode = 0;
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
                m_rosNode = new yarp::os::Node(ros_group.find("rosNodeName").asString());

                if (!m_useGoalFromRosTopic)
                {
                    yInfo() << "goal from ros topic deactivated";
                }
                else
                {

                    yInfo() << "activating ros goal input";

                    if (ros_group.check("goalTopicName"))
                    {
                        if (!m_rosGoalInputPort.topic(ros_group.find("goalTopicName").asString()))
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
                        if (!m_rosCurrentGoal.topic(ros_group.find("currentGoalTopicName").asString()))
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
                        if (!m_localPlan.topic(ros_group.find("localPlanTopicName").asString()))
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
                        if (!m_globalPlan.topic(ros_group.find("globalPlanTopicName").asString()))
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
    m_default_approach_direction    = 180;
    m_default_approach_speed        = 0.2; //ms/s

    m_useGoalFromRosTopic        = false;
    m_publishRosStuff            = false;
    m_obstacle_handler = new obstacles_class(m_rf);
    yInfo("Using following parameters: %s", m_rf.toString().c_str());

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

    if (localization_group.check("robot_frame_id"))             { m_frame_robot_id = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id"))               { m_frame_map_id = localization_group.find("map_frame_id").asString(); }

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

    m_retreat_duration_default = btmp.check("retreat_duration", Value(0.3)).asDouble();

    //open module ports
    string localName = "/robotGoto";

    m_port_commands_output.open((localName + "/control:o").c_str());
    m_port_status_output.open((localName + "/status:o").c_str());
    m_port_speak_output.open((localName + "/speak:o").c_str());
    m_port_gui_output.open((localName + "/gui:o").c_str());

    //localization
    Property loc_options;
    loc_options.put("device", "localization2DClient");
    loc_options.put("local", "/robotGoto/localizationClient");
    loc_options.put("remote", "/localizationServer");
    loc_options.put("period", 10);

    if (m_pLoc.open(loc_options) == false)
    {
        yError() << "Unable to open localization driver";
        return false;
    }
    m_pLoc.view(m_iLoc);
    if (m_iLoc == 0)
    {
        yError() << "Unable to open localization interface";
        return false;
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

    if (m_pLas.open(options) == false)
    {
        yError() << "Unable to open laser driver";
        return false;
    }

    m_pLas.view(m_iLaser);

    if (m_iLaser == 0)
    {
        yError() << "Unable to open laser interface";
        return false;
    }

    if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
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
    b = yarp::os::Network::connect(localName + "/control:o", "/baseControl/control:i");

    /*
    b = Network::connect((localName+"/commands:o").c_str(),"/robot/control:i", "udp", false);
    if (!b) {yError ("Unable to connect the output command port!"); return false;}
    */

    return true;
}

void GotoThread::threadRelease()
{
    if (m_ptf.isValid()) m_ptf.close();
    if (m_pLas.isValid()) m_pLas.close();
    if (m_pLoc.isValid()) m_pLoc.close();

    m_port_target_input.interrupt();
    m_port_target_input.close();
    
    m_port_commands_output.interrupt();
    m_port_commands_output.close();
    
    m_port_status_output.interrupt();
    m_port_status_output.close();
    
    m_port_speak_output.interrupt();
    m_port_speak_output.close();
    
    m_port_gui_output.interrupt();
    m_port_gui_output.close();

    m_rosGoalInputPort.interrupt();
    m_rosGoalInputPort.close();

    m_rosCurrentGoal.interrupt();
    m_rosCurrentGoal.close();

    m_localPlan.interrupt();
    m_localPlan.close();

    m_globalPlan.interrupt();
    m_globalPlan.close();

    if (m_obstacle_handler)
    {
        delete m_obstacle_handler;
        m_obstacle_handler = 0;
    }
}

bool GotoThread::evaluateLocalization()
{
    bool ret = m_iLoc->getCurrentPosition(m_localization_data);
    if (ret)
    {
        m_loc_timeout_counter = 0;
    }
    else
    {
        m_loc_timeout_counter++;
        if (m_loc_timeout_counter>TIMEOUT_MAX) m_loc_timeout_counter = TIMEOUT_MAX;
        return false;
    }
    return true;
}

void GotoThread::evaluateGoalFromTopic()
{
    geometry_msgs_PoseStamped* rosGoalData = m_rosGoalInputPort.read(false);

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
    bool ret = m_iLaser->getLaserMeasurement(m_laser_data);

    if (ret)
    {
        m_las_timeout_counter = 0;
    }
    else
    {
        m_las_timeout_counter++;
        if (m_las_timeout_counter>TIMEOUT_MAX) m_las_timeout_counter = TIMEOUT_MAX;
    }
}

void GotoThread::sendCurrentGoal()
{
    if (!m_publishRosStuff)
    {
        return;
    }
    geometry_msgs_PoseStamped& goal = m_rosCurrentGoal.prepare();
    static int        seq;
    yarp::math::Quaternion q;
    yarp::sig::Vector rpy(3);
    yarp::sig::Matrix m;

    rpy[0]                  = rpy[1] = 0;
    rpy[2]                  = m_target_data.target.theta * DEG2RAD;
    m                       = yarp::math::SE3inv(yarp::math::rpy2dcm(rpy)); 
    q.fromRotationMatrix(m);
    goal.header.frame_id    = m_frame_map_id;
    goal.header.seq         = seq;
    goal.pose.position.x    = m_target_data.target.x;
    goal.pose.position.y    = m_target_data.target.y;
    goal.pose.position.z    = 0;
    goal.pose.orientation.w = q.w();
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();

    m_rosCurrentGoal.write();
    seq++;
}

void GotoThread::publishLocalPlan()
{
    if (m_status != navigation_status_moving || (m_control_out.linear_vel== 0 && m_control_out.angular_vel == 0))
    {
        return;
    }
    else
    {

        static int                seq;
        double                    radius, angle, distance;
        size_t                    pointCount;
        nav_msgs_Path&            path = m_localPlan.prepare();
        yarp::sig::Matrix         map2robotMatrix(4,4);
        map2robotMatrix.zero();
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
        distance = sqrt(pow(m_target_data.target.x - m_localization_data.x, 2) + pow(m_target_data.target.y - m_localization_data.y, 2));
        angle = m_control_out.linear_dir * DEG2RAD;
        path.poses.clear();
        path.poses.resize(pointCount + 1);

        double cs = cos(m_localization_data.theta*DEG2RAD);
        double ss = sin(m_localization_data.theta*DEG2RAD);

        //tranformation matrix from map to robot
        map2robotMatrix[0][0] = cs; map2robotMatrix[0][1] = -ss; map2robotMatrix[0][3] = m_localization_data.x;
        map2robotMatrix[1][0] = ss; map2robotMatrix[1][1] =  cs; map2robotMatrix[0][3] = m_localization_data.y;
        map2robotMatrix[2][2] = 1;
        map2robotMatrix[3][3] = 1;

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

        m_localPlan.write();
        seq++;
    }

}

void GotoThread::getCurrentPos(yarp::dev::Map2DLocation& loc)
{
    loc = m_localization_data;
}

string GotoThread::getMapId()
{
    return m_localization_data.map_id;
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
    if (m_max_lin_speed < 0){ yError() << "Invalid m_max_lin_speed value"; m_max_lin_speed = fabs(m_max_lin_speed); }

    //control saturation.
    //Beware! this test should not inclue the case ==0 to prevent the saturator to override the "do not move" command.
    if      (m_control_out.angular_vel>0)
        m_control_out.angular_vel = std::max(m_min_ang_speed, std::min(m_control_out.angular_vel, m_max_ang_speed));
    else if (m_control_out.angular_vel<0)
        m_control_out.angular_vel = std::max(-m_max_ang_speed, std::min(m_control_out.angular_vel, -m_min_ang_speed));

    //Beware! this test should not inclue the case ==0 to prevent the saturator to override the "do not move" command.
    if      (m_control_out.linear_vel>0)
        m_control_out.linear_vel = std::max(m_min_lin_speed, std::min(m_control_out.linear_vel, m_max_lin_speed));
    else if (m_control_out.linear_vel<0)
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
    m_mutex.wait();
    if (m_useGoalFromRosTopic) evaluateGoalFromTopic();
    evaluateLocalization();
    getLaserData();

    //computes the control action
    m_control_out.zero();

    //gamma is the angle between the current robot heading and the target heading
    double unwrapped_localization_angle = (m_localization_data.theta < 0) ? m_localization_data.theta + 360 : m_localization_data.theta;
    double unwrapped_target_angle = (m_target_data.target.theta       < 0) ? m_target_data.target.theta + 360 : m_target_data.target.theta;
    double gamma                        = unwrapped_target_angle - unwrapped_localization_angle;
    gamma = normalize_angle(gamma);

    //beta is the angle between the current robot position and the target position IN THE WORLD REFERENCE FRAME
    double beta_world = atan2(m_target_data.target.y - m_localization_data.y, m_target_data.target.x - m_localization_data.x) * RAD2DEG;
    //yDebug() << "beta world:" << beta_world;

    //distance is the distance between the current robot position and the target position
    double distance = sqrt(pow(m_target_data.target.x - m_localization_data.x, 2) + pow(m_target_data.target.y - m_localization_data.y, 2));

    //delta is the angle between the current robot position and the target position IN THE ROBOT REFERENCE FRAME
    double beta_robot = (beta_world - m_localization_data.theta);
    beta_robot = normalize_angle(beta_robot);
    //yDebug() << "beta robot:" << beta_robot;
    
    //check for obstacles, always performed
    bool obstacles_in_path = false;
    if (m_las_timeout_counter < 300)
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
            if (yarp::os::Time::now()-m_retreat_starting_time < m_retreat_duration_time)
            {
                m_control_out.linear_dir = m_approach_direction;
                m_control_out.linear_vel = m_approach_speed;
                m_control_out.angular_vel = 0;
            }
            else
            {
                m_status = m_status_after_approach;
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
                        m_control_out.linear_vel = m_gain_lin * distance;
                        m_control_out.linear_dir = beta_robot;
                        m_control_out.angular_vel = m_gain_ang * beta_robot;
                        //===========================
                    }
                    else
                    {
                        //===========================
                        m_control_out.linear_vel = m_gain_lin * distance;
                        m_control_out.linear_dir = 0.0;
                        m_control_out.angular_vel = m_gain_ang * beta_robot;
                        //===========================
                    }
                    //yDebug() << m_control_out.linear_vel;
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
                tmp = m_port_speak_output.prepare();
                tmp.clear();
                tmp = b;
                m_port_speak_output.write();
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
                    tmp = m_port_speak_output.prepare();
                    tmp.clear();
                    tmp = b;
                    m_port_speak_output.write();
                    m_time_ob_obstacle_removal = yarp::os::Time::now();
                }
            }
            else
            {
                if (fabs(current_time - m_time_of_obstacle_detection) > m_obstacle_handler->get_max_time_waiting_for_obstacle_removal())
                {
                    yError ("Failing to recover from obstacle.");
                    m_status = navigation_status_failing;
                }
            }
        break;

        case navigation_status_paused:
            //check if pause is expired
            if (current_time - m_pause_start > m_pause_duration)
            {
                yInfo("pause expired! resuming");
                m_status = navigation_status_moving;
            }
        break;

        case navigation_status_idle:
        case navigation_status_thinking:
        case navigation_status_aborted:
        case navigation_status_failing:
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
    else if (m_status == navigation_status_preparing_before_move)
    {
        //do nothing
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
    m_mutex.post();
}

void GotoThread::sendOutput()
{
    static yarp::os::Stamp stamp;

    stamp.update();
    //send the motors commands and the status to the yarp ports
    if (m_port_commands_output.getOutputCount() > 0)
    {
        Bottle &b = m_port_commands_output.prepare();
        m_port_commands_output.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar speed commands
        b.addDouble(m_control_out.linear_dir);    // angle in deg
        b.addDouble(m_control_out.linear_vel);    // lin_vel in m/s
        b.addDouble(m_control_out.angular_vel);    // ang_vel in deg/s
        b.addDouble(100);
        m_port_commands_output.write();
    }

    if (m_port_status_output.getOutputCount()>0)
    {
        string     string_out;
        string_out = getStatusAsString(m_status);
        Bottle &b = m_port_status_output.prepare();

        m_port_status_output.setEnvelope(stamp);
        b.clear();
        b.addString(string_out.c_str());
        m_port_status_output.write();
    }

    if (m_port_gui_output.getOutputCount() > 0)
    {
        Bottle &b = m_port_gui_output.prepare();
        m_port_gui_output.setEnvelope(stamp);
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
        m_port_gui_output.write();
    }
}

void GotoThread::setNewAbsTarget(yarp::sig::Vector target)
{
    //data is formatted as follows: x, y, angle
    m_target_data.weak_angle = false;
    if (target.size() == 2)
    {
        //if the angle information is missing use as final orientation the direction in which the robot has to move
        double beta = atan2(m_localization_data.y - target[1], m_localization_data.x - target[0])*180.0 / M_PI;
        double beta2 = beta-180;

        if (beta2>+180) beta2 = 360 - beta2;

        if (beta2<-180) beta2 = 360 + beta2;

        target.push_back(beta2);
        m_target_data.weak_angle = true;
    }
    m_target_data.target = Map2DLocation("", target[0], target[1], target[2]);
    m_status = navigation_status_preparing_before_move;
    m_status_after_approach = navigation_status_moving;
    if (m_enable_retreat)
    {
        m_retreat_duration_time    = m_retreat_duration_default;
        m_retreat_starting_time    = yarp::os::Time::now();
        m_approach_direction       = m_default_approach_direction;
        m_approach_speed           = m_default_approach_speed;
    } 
    else
    {
        m_retreat_duration_time = 0;
    }

    yDebug("current pos: abs(%.3f %.3f %.2f)", m_localization_data.x, m_localization_data.y, m_localization_data.theta);
    yDebug("received new target: abs(%.3f %.3f %.2f)", m_target_data.target.x, m_target_data.target.y, m_target_data.target.theta);
    sendCurrentGoal();
}

Map2DLocation GotoThread::getCurrentAbsTarget()
{
    return m_target_data.target;
}

Map2DLocation GotoThread::getCurrentRelTarget()
{
    return Map2DLocation(m_frame_robot_id, m_target_data.target.x - m_localization_data.x, m_target_data.target.y - m_localization_data.y, m_target_data.target.theta - m_localization_data.theta);
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
    m_approach_direction = m_default_approach_direction;
    m_approach_speed     = m_approach_speed;
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
    double a = m_localization_data.theta * DEG2RAD;
    //this is the inverse of the tranformation matrix from world to robot
    m_target_data.target.x     = +target[0] * cos(a) - target[1] * sin(a) + m_localization_data.x;
    m_target_data.target.y     = +target[0] * sin(a) + target[1] * cos(a) + m_localization_data.y;
    m_target_data.target.theta = target[2] + m_localization_data.theta;
    m_status = navigation_status_preparing_before_move;
    m_status_after_approach = navigation_status_moving;
    if (m_enable_retreat)
    { 
        m_retreat_duration_time = m_retreat_duration_default;
        m_approach_direction    = m_default_approach_direction;
        m_approach_speed        = m_default_approach_speed;
    }
    else
    {
        m_retreat_duration_time = 0;
    }
    yInfo("received new target: abs(%.3f %.3f %.2f)", m_target_data.target.x, m_target_data.target.y, m_target_data.target.theta);
    sendCurrentGoal();
}

void GotoThread::approachTarget(double dir, double speed, double time)
{
    m_approach_direction= dir;
    m_approach_speed    = speed;
    m_retreat_duration_time = time;
    m_retreat_starting_time = yarp::os::Time::now();
    m_status            = navigation_status_preparing_before_move;
    m_status_after_approach = navigation_status_idle;
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
        m_pause_duration = secs;
    }
    else
    {
        yInfo( "asked to pause");
        m_pause_duration = 10000000;
    }
    m_status = navigation_status_paused;
    m_pause_start = yarp::os::Time::now();
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
    yDebug("loc timeouts: %d", m_loc_timeout_counter);
    yDebug("las timeouts: %d", m_las_timeout_counter);
    yDebug("status: %s", getStatusAsString(m_status).c_str());
}
