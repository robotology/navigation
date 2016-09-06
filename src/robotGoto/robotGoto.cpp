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

void GotoThread::run()
{
    mutex.wait();
    
    if(useGoalFromRosTopic)
    {
        geometry_msgs_PoseStamped* rosGoalData = rosGoalPort.read(false);

        if(rosGoalData != 0)
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
            v[2] = yarp::math::dcm2rpy(yarp::math::quat2dcm(q))[2];

            setNewAbsTarget(v);
        }
    }
    //data is formatted as follows: x, y, angle (in degrees)
    yarp::sig::Vector *loc = 0;
    
    if (use_localization_from_port)
    {
        loc = port_localization_input.read(false);

        if (loc)
        {
            localization_data   = *loc;
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
        iv.resize(6,0.0);
        pose.resize(6, 0.0);
        bool r = iTf->transformPose(frame_robot_id, frame_map_id, iv, pose);

        if (r)
        {
            //data is formatted as follows: x, y, angle (in degrees)
            localization_data[0] = pose[0];
            localization_data[1] = pose[1];
            localization_data[2] = pose[5] * RAD2DEG;
            loc_timeout_counter  = 0;
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
            yError ("stopping navigation because of localization timeouts!");
            status = navigation_status_aborted;
        }
    }

    yarp::sig::Vector *odm = port_odometry_input.read(false);

    if (odm)
    {
        odometry_data       = *odm;
        odm_timeout_counter = 0;
    }
    else
    {
        if (use_odometry)
            odm_timeout_counter++;

        if (odm_timeout_counter > TIMEOUT_MAX)
            odm_timeout_counter = TIMEOUT_MAX;
    }

    if (odm_timeout_counter>=TIMEOUT_MAX)
    {
        if (status == navigation_status_moving)
        {
            yError ("stopping navigation because of odometry timeouts!");
            status = navigation_status_aborted;
        }
    }

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
        if (las_timeout_counter>TIMEOUT_MAX) las_timeout_counter=TIMEOUT_MAX;
    }

    //computes the control action
    control_out.zero();

    //gamma is the angle between the current robot heading and the target heading
    double unwrapped_localization_angle = (localization_data[2] < 0) ? localization_data[2] + 360 : localization_data[2];
    double unwrapped_target_angle       = (target_data[2] < 0)       ? target_data[2]       + 360  : target_data[2];

    //double gamma  = localization_data[2]-target_data[2];
    double gamma = unwrapped_target_angle - unwrapped_localization_angle;
    if      (gamma >  180)
    {
        gamma -= 360;
    }
    else if (gamma < -180)
    {
        gamma += 360;
    }

    //beta is the angle between the current robot position and the target position
    double beta = atan2 (localization_data[1] - target_data[1], localization_data[0] - target_data[0]) * 180.0 / M_PI;

    //distance is the distance between the current robot position and the target position
    double distance = sqrt(pow(target_data[0] - localization_data[0], 2) + pow(target_data[1] - localization_data[1], 2));

    //compute the control law
    double tmp1 = -180 + (beta - localization_data[2]);

    if (tmp1>360)
    {
        tmp1-=360;
    }

    if (tmp1>180 && tmp1<360)
    {
        tmp1 = tmp1-360;//ADDED LATER
    }

    //yDebug ("%f \n", control[0]);
    control_out[1] = m_gain_lin * distance;
    control_out[2] = m_gain_ang * gamma;
    control_out[0] = tmp1;

    //control saturation
    //yDebug ("%f %f ", control_out[2], control_out[1]);
    if (control_out[2] > 0 )
    {
        if (control_out[2] > +m_max_ang_speed) control_out[2] = +m_max_ang_speed;
        if (control_out[2] < +m_min_ang_speed) control_out[2] = +m_min_ang_speed;
    }
    else
    {
        if (control_out[2] < -m_max_ang_speed) control_out[2] = -m_max_ang_speed;
        if (control_out[2] > -m_min_ang_speed) control_out[2] = -m_min_ang_speed;
    }
   
    if (control_out[1] > 0 )
    {
        if (control_out[1] > +m_max_lin_speed) control_out[1] = +m_max_lin_speed;
        if (control_out[1] < +m_min_lin_speed) control_out[1] = +m_min_lin_speed;
    }
    else
    {
        if (control_out[1] < -m_max_lin_speed) control_out[1] = -m_max_lin_speed;
        if (control_out[1] > -m_min_lin_speed) control_out[1] = -m_min_lin_speed;
    }
    //yDebug ("%f %f \n", control_out[2], control_out[1]);
    
    //check for large rotations: inhibit linear movement, to allow a rotation on place
    if (fabs(gamma)>m_max_gamma_angle)
    {
        control_out[1] = 0;
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

        if (w_f_sat>0.3) w_f_sat=0.3;

        double goal           = control_out[0];
        double goal_corrected = goal * (1-w_f_sat) + correction * (w_f_sat);

        if (enable_obstacles_avoidance)
        {
            //direction is modified in proximity of the obstacles
            control_out[0]  = goal_corrected;

            //speed is reduced in proximity of the obstacles
            double w_f_sat2 = w_f*2.2;

            if (w_f_sat2>0.85) w_f_sat2 = speed_reduction_factor; //CHECK 0.85, put it config file

            control_out[1] = control_out[1] * (1.0-w_f_sat2);
        }
        else
        {
            control_out[0] = goal;
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
            //Update the safety coefficient only if your are MOVING.
            //If you are WAITING_OBSTACLE, use the last valid safety_coeff until the 
            //obstacle has been removed.
            safety_coeff    = control_out[1] / m_max_lin_speed;

            //compute the speed ramp after the removal of an obstacle
            speed_ramp      = (speed_ramp > 1.0) ? 1.0 : speed_ramp;
            control_out[1] *= speed_ramp;
            control_out[2] *= speed_ramp;

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
       control_out[0] = control_out[1] = control_out[2] = 0.0;
    }

    if (enable_retreat && retreat_counter >0)
    {
        control_out[0] = 180;
        control_out[1] = 0.4;
        control_out[2] = 0;
        retreat_counter--;
    }

    sendOutput();
    mutex.post();
}

void GotoThread::sendOutput()
{
    static yarp::os::Stamp stamp;
    stamp.update();
    //send the motors commands and the status to the yarp ports
    if (port_commands_output.getOutputCount()>0)
    {
        Bottle &b = port_commands_output.prepare();
        port_commands_output.setEnvelope(stamp);
        b.clear();
        b.addInt(2);                    // polar commands
        b.addDouble(control_out[0]);    // angle in deg
        b.addDouble(control_out[1]);    // lin_vel in m/s
        b.addDouble(control_out[2]);    // ang_vel in deg/s
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
        b.addDouble(control_out[0]);
        b.addDouble(control_out[1]);
        b.addDouble(control_out[2]);
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

string GotoThread::getNavigationStatus()
{
    return getStatusAsString(status);
}

void GotoThread::printStats()
{
    yDebug( "* robotGoto thread:");
    yDebug( "loc timeouts: %d", loc_timeout_counter);
    yDebug( "odm timeouts: %d", odm_timeout_counter);
    yDebug( "las timeouts: %d", las_timeout_counter);
    yDebug("status: %s", getStatusAsString(status).c_str());
}
