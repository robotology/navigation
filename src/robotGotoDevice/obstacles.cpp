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

#include "obstacles.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

#define DEG2RAD 3.14/180
obstacles_class::obstacles_class(Searchable &rf)
{
    yarp::os::Time::now();
    m_safety_coeff = 1.0;
    m_enable_dynamic_max_distance = false;
    m_max_obstacle_waiting_time = 60.0;
    m_max_obstacle_distance = 0.8;
    m_frontal_blind_angle = 25.0;
    m_speed_reduction_factor = 0.70;
    m_max_detection_distance = 1.5;
    m_min_detection_distance = 0.4;
    m_last_print_time = yarp::os::Time::now();

    /////////////////
    Bottle geometry_group = rf.findGroup("ROBOT_GEOMETRY");
    if (geometry_group.isNull())
    {
        yError() << "Missing ROBOT_GEOMETRY group!";
    }
    bool ff;
    ff = geometry_group.check("robot_radius");
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
    }

    //////////////
    Bottle obstacles_stop_group = rf.findGroup("OBSTACLES_EMERGENCY_STOP");
    if (obstacles_stop_group.isNull())
    {
        yError() << "Missing OBSTACLES_EMERGENCY_STOP group!";
    }

    if (obstacles_stop_group.check("enable_dynamic_max_distance", Value(0)).asInt() == 1)
        m_enable_dynamic_max_distance = true;

    m_max_obstacle_waiting_time = obstacles_stop_group.check("max_waiting_time", Value(60.0)).asDouble();
    m_max_detection_distance = obstacles_stop_group.check("max_detection_distance", Value(1.5)).asDouble();
    m_min_detection_distance = obstacles_stop_group.check("min_detection_distance", Value(0.4)).asDouble();

    //////////////
    Bottle obstacles_avoidance_group = rf.findGroup("OBSTACLES_AVOIDANCE");
    if (obstacles_avoidance_group.isNull())
    {
        yError() << "Missing OBSTACLES_AVOIDANCE group!";
    }

    if (obstacles_avoidance_group.check("frontal_blind_angle"))
        m_frontal_blind_angle = obstacles_avoidance_group.check("frontal_blind_angle", Value(25.0)).asDouble();

    if (obstacles_avoidance_group.check("speed_reduction_factor"))
        m_speed_reduction_factor = obstacles_avoidance_group.check("speed_reduction_factor", Value(0.70)).asDouble();
}

//checks if a point is inside a polygon
int obstacles_class::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
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


bool obstacles_class::compute_obstacle_avoidance(std::vector<LaserMeasurementData>& laser_data)
{
    /*
    double correction = m_angle_f;

    if (correction<0) correction += 180;
    else correction -= 180;

    double w_f_sat = m_w_f;

    if (w_f_sat>0.3) w_f_sat = 0.3;

    double goal = m_control_out[ANG_DIR];
    double goal_corrected = goal * (1 - w_f_sat) + correction * (w_f_sat);

    if (m_enable_obstacles_avoidance)
    {
        //direction is modified in proximity of the obstacles
        m_control_out[ANG_DIR] = goal_corrected;

        //speed is reduced in proximity of the obstacles
        double w_f_sat2 = m_w_f*2.2;

        if (w_f_sat2>0.85) w_f_sat2 = m_speed_reduction_factor; //CHECK 0.85, put it config file

        m_control_out[LIN_VEL] = m_control_out[LIN_VEL] * (1.0 - w_f_sat2);
    }

    m_angle_g = goal_corrected;
*/
    double min_distance = m_max_obstacle_distance;
    double min_angle    = 0.0;

    size_t las_size = laser_data.size();
    if (las_size == 0)
    {
        yError() << "Internal error, invalid laser data struct!";
        return false;
    }

    for (size_t i = 0; i < las_size; i++)
    {
        double curr_d = 0;
        double curr_angle = 0;
        laser_data[i].get_polar(curr_d, curr_angle);

        if (curr_angle >= 0 - m_frontal_blind_angle*DEG2RAD &&
            curr_angle <= 0 + m_frontal_blind_angle*DEG2RAD) continue; //skip frontal obstacles

        if (curr_d < min_distance)
        {
            min_distance = curr_d;
            min_angle    = curr_angle;
        }
    }

    m_angle_f = min_angle;
    m_angle_t = m_angle_f + 90.0;
    m_w_f = (1 - (min_distance / m_max_obstacle_distance)) / 2;
    m_w_t = 0;
    return true;
}

bool obstacles_class::check_obstacles_in_path(std::vector<LaserMeasurementData>& laser_data)
{
    static double last_time_error_message = 0;
    int laser_obstacles  = 0;
    double goal_distance = 1000; //TO BE COMPLETED

    //compute the polygon
    double vertx[4];
    double verty[4];
    double theta              = 0.0;
    //the following 90 degrees rotation is needed to perform the following reference frame rotation.
    //the reference frame of the robot is the one shown on the right.
    //      Y                 X
    //      |       -->       |
    //      O--X           Y--O
    double ctheta             = cos(theta-1.5707);
    double stheta             = sin(theta-1.5707);
    double detection_distance = m_min_detection_distance;

    if (m_enable_dynamic_max_distance)
    {
        //detection_distance is increased from min to max as the velocity of the robot increases
        detection_distance = m_max_detection_distance * m_safety_coeff;
    }

    //an obstacle farther than m_max_detection_distance is always ignored
    if (detection_distance>m_max_detection_distance)
        detection_distance = m_max_detection_distance;

    //an obstacle nearer than m_min_detection_distance is always detected
    if (detection_distance<m_min_detection_distance)
        detection_distance = m_min_detection_distance;

    vertx[0] = (-m_robot_radius) * ctheta + detection_distance * (-stheta);
    verty[0] = (-m_robot_radius) * stheta + detection_distance * ctheta;
    vertx[1] = (+m_robot_radius) * ctheta + detection_distance * (-stheta);
    verty[1] = (+m_robot_radius) * stheta + detection_distance * ctheta;
    vertx[2] = +m_robot_radius  * ctheta;
    verty[2] = +m_robot_radius  * stheta;
    vertx[3] = -m_robot_radius  * ctheta;
    verty[3] = -m_robot_radius  * stheta;

    size_t las_size = laser_data.size();

    if (las_size == 0)
    {
        yError() << "Internal error, invalid laser data struct!";
        return false;
    }

    for (size_t i = 0; i < las_size; i++)
    {
        double d = 0;
        double angle = 0;
        laser_data[i].get_polar(d, angle);

        if (d < m_robot_radius)
        {
            laser_obstacles++;
            if (yarp::os::Time::now() - last_time_error_message > 0.3)
            {
                yError("obstacles on the platform");
                last_time_error_message = yarp::os::Time::now();
            }
            continue;
        }

        double px = 0;
        double py = 0;
        laser_data[i].get_cartesian(px, py);

        if (pnpoly(4,vertx,verty,px,py)>0)
        {
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

    //prevent noise to be detected as an obstacle;
    if (laser_obstacles>=2)
    {
        if (yarp::os::Time::now() - m_last_print_time > 1.0)
        {
            yWarning("obstacles detected");
            m_last_print_time = yarp::os::Time::now();
        }
        return true;
    }

    //no obstacles found
    return false;
}


double obstacles_class::get_max_time_waiting_for_obstacle_removal()
{
    return m_max_obstacle_waiting_time;
}

void obstacles_class::set_safety_coeff(double val)
{
    m_safety_coeff = val;
}

