/* 
 * Copyright (C)2017  iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Subscriber.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LockGuard.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/INavigation2D.h>
#include <string>
#include <math.h>
#include <visualization_msgs_MarkerArray.h>
#include <geometry_msgs_PoseStamped.h>
#include <nav_msgs_Path.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

typedef yarp::os::Subscriber<geometry_msgs_PoseStamped> rosGoalSubscriber;
typedef yarp::os::Publisher<geometry_msgs_PoseStamped>  rosGoalPublisher;
typedef yarp::os::Publisher<nav_msgs_Path>              rosPathPublisher;

class obstacles_class
{
private:
    double m_robot_radius;        //m
    double m_robot_laser_x;       //m
    double m_robot_laser_y;       //m
    double m_robot_laser_t;       //deg

    double m_last_print_time;
public:
    //obstacles_emergency_stop block
    double               m_max_obstacle_distance;
    double               m_frontal_blind_angle;
    double               m_speed_reduction_factor;
    double               m_angle_f;
    double               m_angle_t;
    double               m_angle_g;
    double               m_w_f;
    double               m_w_t;
    double               m_w_g;

    //obstacle avoidance block
    bool                 m_enable_dynamic_max_distance;
    double               m_max_obstacle_waiting_time;
    double               m_safety_coeff;
    double               m_max_detection_distance;
    double               m_min_detection_distance;

public:
    obstacles_class(ResourceFinder  &rf);
    bool check_obstacles_in_path(std::vector<LaserMeasurementData>& laser_data);
    bool compute_obstacle_avoidance(std::vector<LaserMeasurementData>& laser_data);
    double get_max_time_waiting_for_obstacle_removal();
    void set_safety_coeff(double val);

private:
    int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
};

#endif
