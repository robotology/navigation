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

#include "robotGoto.h"

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

    yError("Unknown status of inner controller: '%d'!", status);
    return std::string("unknown");
}

GotoThread::GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
            RateThread(_period),
            rf(_rf),
            robotCtrl_options (options)
{
    yarp::os::Time::now();
    status                          = navigation_status_idle;
    loc_timeout_counter             = TIMEOUT_MAX;
    odm_timeout_counter             = TIMEOUT_MAX;
    las_timeout_counter             = TIMEOUT_MAX;
    localization_data.resize(3,0.0);
    retreat_counter                 = 0;
    safety_coeff                    = 1.0;
    enable_obstacles_emergency_stop = false;
    enable_obstacles_avoidance      = false;
    enable_dynamic_max_distance     = false;
    enable_retreat                  = false;
    retreat_duration                = 300;
    control_out.resize(3,0.0);
    pause_start                     = 0;
    pause_duration                  = 0;
    max_obstacle_wating_time        = 60.0;
    max_obstacle_distance           = 0.8;
    frontal_blind_angle             = 25.0;
    speed_reduction_factor          = 0.70;
    max_detection_distance          = 1.5;
    min_detection_distance          = 0.4;
    obstacle_removal_time           = 0.0;
    iLaser                          = 0;
    iTf                             = 0;
    min_laser_angle                 = 0;
    max_laser_angle                 = 0;
    robot_radius                    = 0;
    robot_laser_x                   = 0;
    robot_laser_y                   = 0;
    robot_laser_t                   = 0;
    laser_data                      = 0;
    rosNode                         = 0;
}

bool GotoThread::rosInit(const yarp::os::Bottle& ros_group)
{
    if(ros_group.check("useGoalFromRosTopic") || ros_group.check("publishRosStuff"))
    {
        useGoalFromRosTopic  = ros_group.find("useGoalFromRosTopic").asBool();
        publishRosStuff      = ros_group.find("publishRosStuff").asBool();

        if(useGoalFromRosTopic || publishRosStuff)
        {
            if (ros_group.check("rosNodeName"))
            {
                rosNode = new yarp::os::Node( ros_group.find("rosNodeName").asString() );

                if (!useGoalFromRosTopic)
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

                if (!publishRosStuff)
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
    robot_is_holonomic              = false;
    m_default_max_gamma_angle       = m_max_gamma_angle = 5;
    m_default_gain_ang = m_gain_ang = 0.05;
    m_default_gain_lin = m_gain_lin = 0.1;
    m_default_max_lin_speed         = m_max_lin_speed = 0.9;  //m/s
    m_default_max_ang_speed         = m_max_ang_speed = 10.0; //deg/s
    m_default_max_lin_speed         = m_min_lin_speed = 0.0;  //m/s
    m_default_max_ang_speed         = m_min_ang_speed = 0.0; //deg/s
    m_default_goal_tolerance_lin    = m_goal_tolerance_lin = 0.05;
    m_default_goal_tolerance_ang    = m_goal_tolerance_ang = 0.6;

    use_odometry               = true;
    use_localization_from_port = false;
    use_localization_from_tf   = false;
    useGoalFromRosTopic        = false;
    publishRosStuff            = false;
    yInfo("Using following paramters: %s", rf.toString().c_str());

    Bottle ros_group = rf.findGroup("ROS");
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

    Bottle trajectory_group = rf.findGroup("ROBOT_TRAJECTORY");
    if (trajectory_group.isNull())
    {
        yError() << "Missing ROBOT_TRAJECTORY group!";
        return false;
    }

    if (trajectory_group.check("robot_is_holonomic")) { robot_is_holonomic           = trajectory_group.find("robot_is_holonomic").asInt() == 1; }
    if (trajectory_group.check("max_gamma_angle"))    { m_default_max_gamma_angle    = m_max_gamma_angle    = trajectory_group.find("max_gamma_angle").asDouble(); }
    if (trajectory_group.check("ang_speed_gain"))     { m_default_gain_ang           = m_gain_ang           = trajectory_group.find("ang_speed_gain").asDouble(); }
    if (trajectory_group.check("lin_speed_gain"))     { m_default_gain_lin           = m_gain_lin           = trajectory_group.find("lin_speed_gain").asDouble(); }
    if (trajectory_group.check("max_lin_speed"))      { m_default_max_lin_speed      = m_max_lin_speed      = trajectory_group.find("max_lin_speed").asDouble(); }
    if (trajectory_group.check("max_ang_speed"))      { m_default_max_ang_speed      = m_max_ang_speed      = trajectory_group.find("max_ang_speed").asDouble(); }
    if (trajectory_group.check("min_lin_speed"))      { m_default_max_ang_speed      = m_min_lin_speed      = trajectory_group.find("min_lin_speed").asDouble(); }
    if (trajectory_group.check("min_ang_speed"))      { m_default_max_ang_speed      = m_min_ang_speed      = trajectory_group.find("min_ang_speed").asDouble(); }
    if (trajectory_group.check("goal_tolerance_lin")) { m_default_goal_tolerance_lin = m_goal_tolerance_lin = trajectory_group.find("goal_tolerance_lin").asDouble(); }
    if (trajectory_group.check("goal_tolerance_ang")) { m_default_goal_tolerance_lin = m_goal_tolerance_ang = trajectory_group.find("goal_tolerance_ang").asDouble(); }

    Bottle geometry_group = rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
        return false;
    }
    Bottle localization_group = rf.findGroup("LOCALIZATION");
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
        robot_radius  = geometry_group.find("robot_radius").asDouble();
        robot_laser_x = geometry_group.find("laser_pos_x").asDouble();
        robot_laser_y = geometry_group.find("laser_pos_y").asDouble();
        robot_laser_t = geometry_group.find("laser_pos_theta").asDouble();
    }
    else
    {
        yError() << "Invalid/missing parameter in ROBOT_GEOMETRY group";
        return false;
    }

    if (localization_group.check("use_odometry"))               { use_odometry               = (localization_group.find("use_odometry").asInt() == 1); }
    if (localization_group.check("use_localization_from_port")) { use_localization_from_port = (localization_group.find("use_localization_from_port").asInt() == 1); }
    if (localization_group.check("use_localization_from_tf"))   { use_localization_from_tf   = (localization_group.find("use_localization_from_tf").asInt() == 1); }
    if (localization_group.check("robot_frame_id"))             { this->frame_robot_id       = localization_group.find("robot_frame_id").asString(); }
    if (localization_group.check("map_frame_id"))               { this->frame_map_id         = localization_group.find("map_frame_id").asString(); }
    if (use_localization_from_port == true && use_localization_from_tf == true)
    {
        yError() << "`use_localization_from_tf` and `use_localization_from_port` cannot be true simulteneously!";
        return false;
    }

    Bottle btmp;
    btmp = rf.findGroup("RETREAT_OPTION");

    if (btmp.check("enable_retreat", Value(0)).asInt() == 1)
        enable_retreat = true;

    retreat_duration = btmp.check("retreat_duration", Value(300)).asInt();

    btmp = rf.findGroup("OBSTACLES_EMERGENCY_STOP");

    if (btmp.check("enable_obstacles_emergency_stop", Value(0)).asInt() == 1)
        enable_obstacles_emergency_stop = true;

    if (btmp.check("enable_dynamic_max_distance", Value(0)).asInt() == 1)
        enable_dynamic_max_distance = true;

    max_obstacle_wating_time = btmp.check("max_wating_time", Value(60.0)).asDouble();
    max_detection_distance   = btmp.check("max_detection_distance", Value(1.5)).asDouble();
    min_detection_distance   = btmp.check("min_detection_distance", Value(0.4)).asDouble();

    btmp = rf.findGroup("OBSTACLES_AVOIDANCE");

    if (btmp.check("enable_obstacles_avoidance", Value(0)).asInt() == 1)
        enable_obstacles_avoidance = true;

    if (btmp.check("frontal_blind_angle"))
        frontal_blind_angle = btmp.check("frontal_blind_angle", Value(25.0)).asDouble();

    if (btmp.check("speed_reduction_factor"))
        speed_reduction_factor = btmp.check("speed_reduction_factor", Value(0.70)).asDouble();

    //open module ports
    string localName = "/robotGoto";

    port_commands_output.open((localName + "/control:o").c_str());
    port_status_output.open((localName + "/status:o").c_str());
    port_odometry_input.open((localName + "/odometry:i").c_str());
    port_speak_output.open((localName + "/speak:o").c_str());
    port_gui_output.open((localName + "/gui:o").c_str());

    //localization
    if (use_localization_from_port)
    {
        port_localization_input.open((localName + "/localization:i").c_str());
    }

    if (use_localization_from_tf)
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
    Bottle laserBottle = rf.findGroup("LASER");

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

    if (iLaser->getScanLimits(min_laser_angle, max_laser_angle) == false)
    {
        yError() << "Unable to obtain laser scan limits";
        return false;
    }

    laser_angle_of_view = fabs(min_laser_angle) + fabs(max_laser_angle);

    //automatic connections for debug
    bool b = true;
    b = yarp::os::Network::connect("/robot/laser:o", "/yarpLaserScannerGui/laser:i");
    b = yarp::os::Network::connect("/robotGoto/gui:o", "/yarpLaserScannerGui/nav_display:i");

    //automatic port connections
    b = yarp::os::Network::connect("/baseControl/odometry:o", localName + "/odometry:i");
    b = yarp::os::Network::connect(localName + "/control:o", "/baseControl/control:i");

    yarp::os::Time::delay(0.5);

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
}

//checks if a point is inside a polygon
int GotoThread::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++)
  {
    if (( (verty[i]>testy) != (verty[j]>testy) ) &&
          (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
    {
       c = !c;
    }
  }
  return c;
}


bool GotoThread::compute_obstacle_avoidance()
{
    double min_distance = max_obstacle_distance;
    double min_angle    = 0.0;

    if (laser_data == 0)
    {
        yError() << "Internal error, invalid laser data struct!";
        return false;
    }

    size_t las_size = laser_data->size();

    for (size_t i = 0; i < las_size; i++)
    {
        double curr_d     = laser_data->get_distance(i);
        double curr_angle = laser_data->get_angle(i);
        size_t angle_t    = (size_t)(4.0 * frontal_blind_angle);

        if (i>=540-angle_t && i<=540+angle_t) continue; //skip frontalobstacles

        if (curr_d < min_distance)
        {
            min_distance = curr_d;
            min_angle    = curr_angle;
        }
    }

    angle_f = min_angle;
    angle_t = angle_f+90.0;
    w_f     = (1-(min_distance/max_obstacle_distance))/2;
    w_t     = 0;
    return true;
}

bool GotoThread::check_obstacles_in_path()
{
    int laser_obstacles  = 0;
    double goal_distance = 1000; //TO BE COMPLETED

    //compute the polygon
    double vertx[4];
    double verty[4];
    double theta              = 0.0;
    double ctheta             = cos(theta);
    double stheta             = sin(theta);
    double detection_distance = 1.5;

    if(enable_dynamic_max_distance)
        detection_distance = max_detection_distance * safety_coeff;
    else
        detection_distance = max_detection_distance;

    if (detection_distance<min_detection_distance)
        detection_distance = min_detection_distance;

    vertx[0] = (-robot_radius) * ctheta + detection_distance * (-stheta);
    verty[0] = (-robot_radius) * stheta + detection_distance * ctheta;
    vertx[1] = (+robot_radius) * ctheta + detection_distance * (-stheta);
    verty[1] = (+robot_radius) * stheta + detection_distance * ctheta;
    vertx[2] =  +robot_radius  * ctheta;
    verty[2] =  +robot_radius  * stheta;
    vertx[3] =  -robot_radius  * ctheta;
    verty[3] =  -robot_radius  * stheta;

    if (laser_data == 0)
    {
        yError() << "Internal error, invalid laser data struct!";
        return false;
    }

    size_t las_size = laser_data->size();

    for (size_t i = 0; i < las_size; i++)
    {
        double d = laser_data->get_distance(i);

        if (d < robot_radius)
        {
            laser_obstacles++;
            yError("obstacles on the platform");
            continue;
        }

        double px = laser_data->get_x(i);
        double py = laser_data->get_y(i);

        if (pnpoly(4,vertx,verty,px,py)>0)
        {
            double d = laser_data->get_distance(i);

            //if (laser_data.get_distance(i) < goal_distance)
            if (d < goal_distance)
            {
                laser_obstacles++;
                //yError("obstacles on the path");
                continue;
            }
            else
            {
                //yError("obstacles on the path, but goal is near");
                continue;
            }
        }
    }

    //prevent noise to be detected as an obtacle;
    if (laser_obstacles>=2)
    {
        yWarning("obstacles detected");
        return true;
    }

    //no obstacles found
    return false;
}

void GotoThread::evaluateLocalization()
{
    //data is formatted as follows: x, y, angle (in degrees)
    yarp::sig::Vector *loc = 0;

    if (use_localization_from_port)
    {
        loc = port_localization_input.read(false);

        if (loc)
        {
            localization_data = *loc;
            loc_timeout_counter = 0;
        }
        else
        {
            loc_timeout_counter++;
            if (loc_timeout_counter>TIMEOUT_MAX) loc_timeout_counter = TIMEOUT_MAX;
        }
    }
    else if (use_localization_from_tf)
    {
        yarp::sig::Vector iv;
        yarp::sig::Vector pose;
        iv.resize(6, 0.0);
        pose.resize(6, 0.0);
        bool r = iTf->transformPose(frame_robot_id, frame_map_id, iv, pose);

        if (r)
        {
            //data is formatted as follows: x, y, angle (in degrees)
            localization_data[X]     = pose[0];
            localization_data[Y]     = pose[1];
            localization_data[ANGLE] = pose[5] * RAD2DEG;
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
        if (status == navigation_status_moving)
        {
            yError("stopping navigation because of localization timeouts!");
            status = navigation_status_aborted;
        }
    }

    yarp::sig::Vector *odm = port_odometry_input.read(false);

    if (odm)
    {
        odometry_data = *odm;
        odm_timeout_counter = 0;
    }
    else
    {
        if (use_odometry)
            odm_timeout_counter++;

        if (odm_timeout_counter > TIMEOUT_MAX)
            odm_timeout_counter = TIMEOUT_MAX;
    }

    if (odm_timeout_counter >= TIMEOUT_MAX)
    {
        if (status == navigation_status_moving)
        {
            yError("stopping navigation because of odometry timeouts!");
            status = navigation_status_aborted;
        }
    }
}

void GotoThread::evaluateGoalFromTopic()
{
    if (useGoalFromRosTopic)
    {
        geometry_msgs_PoseStamped* rosGoalData = rosGoalInputPort.read(false);

        if (rosGoalData != 0)
        {
            yInfo() << "received a goal from ros topic";
            yarp::sig::Vector v(3);
            yarp::sig::Vector q(4);

            q[0] = rosGoalData->pose.orientation.w;
            q[1] = rosGoalData->pose.orientation.x;
            q[2] = rosGoalData->pose.orientation.y;
            q[3] = rosGoalData->pose.orientation.z;

            v[0] = rosGoalData->pose.position.x;
            v[1] = rosGoalData->pose.position.y;
            v[2] = -yarp::math::dcm2rpy(yarp::math::quat2dcm(q))[2] * RAD2DEG;

            setNewAbsTarget(v);
        }
    }
}

void GotoThread::getLaserData()
{
    yarp::sig::Vector scan;
    bool ret = iLaser->getMeasurementData(scan);

    if (ret)
    {
        if (laser_data == 0)
        {
            laser_data = new laser_type(scan.size(), this->laser_angle_of_view);
            laser_data->set_laser_position(robot_laser_x, robot_laser_y, robot_laser_t);
        }
        laser_data->set_cartesian_laser_data(scan);
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
    if(!publishRosStuff)
    {
        return;
    }
    geometry_msgs_PoseStamped& goal = rosCurrentGoal.prepare();
    static int        seq;
    yarp::sig::Vector q(4);
    yarp::sig::Vector rpy(3);

    rpy[0]                  = rpy[1] = 0;
    rpy[2]                  = target_data[ANGLE] * DEG2RAD;
    q                       = yarp::math::dcm2quat(yarp::math::SE3inv(yarp::math::rpy2dcm(rpy)));
    goal.header.frame_id    = frame_map_id;
    goal.header.seq         = seq;
    goal.pose.position.x    = target_data[X];
    goal.pose.position.y    = target_data[Y];
    goal.pose.position.z    = 0;
    goal.pose.orientation.w = q[0];
    goal.pose.orientation.x = q[1];
    goal.pose.orientation.y = q[2];
    goal.pose.orientation.z = q[3];

    rosCurrentGoal.write();
    seq++;
}

void GotoThread::publishLocalPlan()
{

    if(control_out[LIN_VEL] == 0 && control_out[ANG_VEL] == 0)
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
        path.header.frame_id   = frame_map_id;
        path.header.seq        = seq;
        path.header.stamp.sec  = int(yarp::os::Time::now());;
        path.header.stamp.nsec = (yarp::os::Time::now() - int(yarp::os::Time::now())) * 1000000;

        //drawing data
        pointCount = 10;
        radius     = 0.7;
        distance   = sqrt(pow(target_data[X] - localization_data[X], 2) + pow(target_data[Y] - localization_data[Y], 2));
        angle      = control_out[ANG_MOM] * DEG2RAD;
        iTf->getTransform(frame_robot_id, frame_map_id, map2robotMatrix);
        path.poses.clear();
        path.poses.resize(pointCount + 1);

        for(size_t i = 0; i < pointCount + 1; i++)
        {
            pos[0]                  = control_out[LIN_VEL] ? distance / pointCount * i : radius * cos(angle / pointCount * i);
            pos[1]                  = control_out[LIN_VEL] ? 0                         : radius * sin(angle / pointCount * i);
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
    v.resize(localization_data.size());
    v = localization_data;
}

string GotoThread::getMapId()
{
    return frame_map_id;
}

void GotoThread::run()
{
    mutex.wait();
    evaluateGoalFromTopic();
    yarp::os::Time::delay(0.05);
    evaluateLocalization();
    getLaserData();

    //computes the control action
    control_out.zero();

    //gamma is the angle between the current robot heading and the target heading
    double unwrapped_localization_angle = (localization_data[ANGLE] < 0) ? localization_data[ANGLE] + 360  : localization_data[ANGLE];
    double unwrapped_target_angle       = (target_data[ANGLE] < 0)       ? target_data[ANGLE]       + 360  : target_data[ANGLE];
    double gamma                        = unwrapped_target_angle - unwrapped_localization_angle;

    if      (gamma >  180)
    {
        gamma -= 360;
    }
    else if (gamma < -180)
    {
        gamma += 360;
    }

    //beta is the angle between the current robot position and the target position
    double beta = atan2 ( target_data[Y] - localization_data[Y], target_data[X] - localization_data[X]) * RAD2DEG;

    //distance is the distance between the current robot position and the target position
    double distance = sqrt(pow(target_data[X] - localization_data[X], 2) + pow(target_data[Y] - localization_data[Y], 2));

    //compute the control law
    double tmp1 = (beta - localization_data[ANGLE]);

    if (tmp1>360)
    {
        tmp1-=360;
    }

    if (tmp1>180 && tmp1<360)
    {
        tmp1 = tmp1-360;//ADDED LATER
    }

    if (tmp1<-180 && tmp1>-360)
    {
        tmp1 = tmp1+360;//ADDED LATER
    }

    //yDebug ("%f \n", control[0]);
    control_out[LIN_VEL] = m_gain_lin * distance;
    if(distance <= m_goal_tolerance_lin)
    {
        control_out[ANG_VEL] = m_gain_ang * gamma;
        control_out[LIN_VEL] = 0;
    }
    else
    {
        control_out[ANG_VEL] = m_gain_ang * (tmp1);
    }
    control_out[ANG_MOM] = tmp1;

    //control saturation
    //yDebug ("%f %f ", control_out[ANG_VEL], control_out[LIN_VEL]);
    if (control_out[ANG_VEL] > 0 )
    {
        if (control_out[ANG_VEL] > +m_max_ang_speed) control_out[ANG_VEL] = +m_max_ang_speed;
        if (control_out[ANG_VEL] < +m_min_ang_speed) control_out[ANG_VEL] = +m_min_ang_speed;
    }
    else
    {
        if (control_out[ANG_VEL] < -m_max_ang_speed) control_out[ANG_VEL] = -m_max_ang_speed;
        if (control_out[ANG_VEL] > -m_min_ang_speed) control_out[ANG_VEL] = -m_min_ang_speed;
    }
   
    if (control_out[LIN_VEL] > 0 )
    {
        if (control_out[LIN_VEL] > +m_max_lin_speed) control_out[LIN_VEL] = +m_max_lin_speed;
        if (control_out[LIN_VEL] < +m_min_lin_speed) control_out[LIN_VEL] = +m_min_lin_speed;
    }
    else
    {
        if (control_out[LIN_VEL] < -m_max_lin_speed) control_out[LIN_VEL] = -m_max_lin_speed;
        if (control_out[LIN_VEL] > -m_min_lin_speed) control_out[LIN_VEL] = -m_min_lin_speed;
    }
    //yDebug ("%f %f \n", control_out[ANG_VEL], control_out[LIN_VEL]);
    
    //check for large rotations: inhibit linear movement, to allow a rotation on place
    if (fabs(beta - localization_data[ANGLE])>m_max_gamma_angle)
    {
        control_out[LIN_VEL] = 0;
    }

    //check for obstacles, always performed
    bool obstacles_in_path = false;

    if (las_timeout_counter < 300)
    {
        obstacles_in_path = check_obstacles_in_path();
        compute_obstacle_avoidance();

        double correction = angle_f;

        if (correction<0) correction += 180;
        else correction -= 180;

        double w_f_sat = w_f;

        if (w_f_sat>0.3) w_f_sat = 0.3;

        double goal           = control_out[ANG_MOM];
        double goal_corrected = goal * (1-w_f_sat) + correction * (w_f_sat);

        if (enable_obstacles_avoidance)
        {
            //direction is modified in proximity of the obstacles
            control_out[ANG_MOM]  = goal_corrected;

            //speed is reduced in proximity of the obstacles
            double w_f_sat2 = w_f*2.2;

            if (w_f_sat2>0.85) w_f_sat2 = speed_reduction_factor; //CHECK 0.85, put it config file

            control_out[LIN_VEL] = control_out[LIN_VEL] * (1.0-w_f_sat2);
        }
        else
        {
            control_out[ANG_MOM] = goal;
        }
        angle_g = goal_corrected;
    }
    else
    {
        if (status == navigation_status_moving)
        {
        }
    }

    double current_time = yarp::os::Time::now();
    double speed_ramp   = (current_time-obstacle_removal_time)/2.0;

    switch (status)
    {
    case navigation_status_moving:
        int a;
        //Update the safety coefficient only if your are MOVING.
        //If you are WAITING_OBSTACLE, use the last valid safety_coeff until the
        //obstacle has been removed.
        safety_coeff    = control_out[LIN_VEL] / m_max_lin_speed;

        //compute the speed ramp after the removal of an obstacle
        speed_ramp      = (speed_ramp > 1.0) ? 1.0 : speed_ramp;
        control_out[LIN_VEL] *= speed_ramp;
        control_out[ANG_VEL] *= speed_ramp;

        if (target_data.weak_angle)
        {
            //check if the goal has been reached in position but not in orientation
            if (fabs(distance) < m_goal_tolerance_lin)
            {
                status = navigation_status_goal_reached;
                yInfo ("Goal reached!");
            }
        }
        else
        {
            //check if the goal has been reached in both position and orientation
            //yDebug() << fabs(distance) << fabs(gamma);
            if (fabs(distance) < m_goal_tolerance_lin && fabs(gamma) < m_goal_tolerance_ang)
            {
                status = navigation_status_goal_reached;
                yInfo("Goal reached!");
            }

        }

        // check if you have to stop beacuse of an obstacle
        if  (enable_obstacles_emergency_stop && obstacles_in_path)
        {
            yInfo ("Obstacles detected, stopping");
            Bottle b, tmp;

            status        = navigation_status_waiting_obstacle;
            obstacle_time = current_time;

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
            if (fabs(current_time-obstacle_time) > 1.0)
            {

                yInfo ("Obstacles removed, thank you");
                Bottle b, tmp;
                status = navigation_status_moving;

                b.addString("Obstacles removed, thank you");
                tmp = port_speak_output.prepare();
                tmp.clear();
                tmp = b;
                port_speak_output.write();
                obstacle_removal_time = yarp::os::Time::now();
            }
        }
        else
        {
            if (fabs(current_time - obstacle_time) > max_obstacle_wating_time)
            {
                yError ("failed to recover from obstacle, goal aborted");
                status = navigation_status_aborted;
            }
        }
    break;

    case navigation_status_paused:
        //check if pause is expired
        double current_time = yarp::os::Time::now();
        if (current_time - pause_start > pause_duration)
        {
            yInfo("pause expired! resuming");
            status = navigation_status_moving;
        }
    break;
    }

    if (status != navigation_status_moving)
    {
       control_out[ANG_MOM] = control_out[LIN_VEL] = control_out[ANG_VEL] = 0.0;
    }

    if (enable_retreat && retreat_counter > 0)
    {
        control_out[ANG_MOM] = 180;
        control_out[LIN_VEL] = 0.4;
        control_out[ANG_VEL] = 0;
        retreat_counter--;
    }

    publishLocalPlan();
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
        b.addInt(2);                    // polar commands
        b.addDouble(control_out[ANG_MOM]);    // angle in deg
        b.addDouble(control_out[LIN_VEL]);    // lin_vel in m/s
        b.addDouble(control_out[ANG_VEL]);    // ang_vel in deg/s
        port_commands_output.write();
    }

    if (port_status_output.getOutputCount()>0)
    {
        string     string_out;
        string_out = getStatusAsString(status);
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
        b.addDouble(control_out[ANG_MOM]);
        b.addDouble(control_out[LIN_VEL]);
        b.addDouble(control_out[ANG_VEL]);
        b.addDouble(angle_f);
        b.addDouble(angle_t);
        b.addDouble(w_f);
        b.addDouble(w_t);
        b.addDouble(max_obstacle_distance);
        b.addDouble(angle_g);
        port_gui_output.write();
    }
}

void GotoThread::setNewAbsTarget(yarp::sig::Vector target)
{
    //data is formatted as follows: x, y, angle
    target_data.weak_angle = false;
    if (target.size() == 2)
    {
        //if the angle information is missing use as final orientation the direction in which the robot has to move
        double beta  = atan2 (localization_data[1]-target[1],localization_data[0]-target[0])*180.0/M_PI;
        double beta2 = beta-180;

        if (beta2>+180) beta2 = 360 - beta2;

        if (beta2<-180) beta2 = 360 + beta2;

        target.push_back(beta2);
        target_data.weak_angle = true;
    }
    target_data.target = target;
    status             = navigation_status_moving;
    retreat_counter    = retreat_duration;

    yDebug ( "current pos: abs(%.3f %.3f %.2f)", localization_data[0], localization_data[1], localization_data[2]);
    yDebug ( "received new target: abs(%.3f %.3f %.2f)", target_data[0], target_data[1], target_data[2]);
    sendCurrentGoal();
}

void GotoThread::resetParamsToDefaultValue()
{
    m_max_gamma_angle    = m_default_max_gamma_angle;
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
    target_data.weak_angle = false;
    if (target.size() == 2)
    {
        target.push_back(0.0);
        target_data.weak_angle = true;
    }
    double a        = localization_data[2] * DEG2RAD;
    //this is the inverse of the tranformation matrix from world to robot
    target_data[0]  = +target[0] * cos(a) - target[1] * sin(a) + localization_data[0];
    target_data[1]  = +target[0] * sin(a) + target[1] * cos(a) + localization_data[1];
    target_data[2]  =  target[2] + localization_data[2];
    status          = navigation_status_moving;
    retreat_counter = retreat_duration;
    yInfo ( "received new target: abs(%.3f %.3f %.2f)", target_data[0], target_data[1], target_data[2]);
    sendCurrentGoal();
}

void GotoThread::pauseMovement(double secs)
{
    if (status == navigation_status_paused)
    {
        yWarning ( "already in pause!");
        return;
    }
    if (status != navigation_status_moving)
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
    status      = navigation_status_paused;
    pause_start = yarp::os::Time::now();
}

void GotoThread::resumeMovement()
{
    yInfo( "asked to resume movement");
    status = navigation_status_moving;
}

void GotoThread::stopMovement()
{
    yInfo( "asked to stop");
    status = navigation_status_idle;
}

string GotoThread::getNavigationStatusAsString()
{
    return getStatusAsString(status);
}

int GotoThread::getNavigationStatusAsInt()
{
    return status;
}

void GotoThread::printStats()
{
    yDebug( "* robotGoto thread:");
    yDebug( "loc timeouts: %d", loc_timeout_counter);
    yDebug( "odm timeouts: %d", odm_timeout_counter);
    yDebug( "las timeouts: %d", las_timeout_counter);
    yDebug("status: %s", getStatusAsString(status).c_str());
}
