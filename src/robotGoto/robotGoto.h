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

#ifndef COMPASS_THREAD_H
#define COMPASS_THREAD_H

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
#include <yarp/dev/ILocalization2D.h>
#include <string>
#include <math.h>
#include <visualization_msgs_MarkerArray.h>
#include <geometry_msgs_PoseStamped.h>
#include <nav_msgs_Path.h>
#include "obstacles.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

typedef yarp::os::Subscriber<geometry_msgs_PoseStamped> rosGoalSubscriber;
typedef yarp::os::Publisher<geometry_msgs_PoseStamped>  rosGoalPublisher;
typedef yarp::os::Publisher<nav_msgs_Path>              rosPathPublisher;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define TIMEOUT_MAX 300
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

struct target_type
{
    yarp::dev::Map2DLocation target;
    bool                     weak_angle;

    target_type() {weak_angle = false;}
};

class GotoThread: public yarp::os::RateThread
{
    /////////////////////////////////////
    //PROPERTIES
    ////////////////////////////////////
public:
    bool   m_enable_retreat;
    int    m_retreat_duration;

    //robot properties
    bool   m_robot_is_holonomic;
    bool   m_enable_obstacles_emergency_stop;
    bool   m_enable_obstacles_avoidance;
    double m_robot_radius;        //m
    double m_robot_laser_x;       //m
    double m_robot_laser_y;       //m
    double m_robot_laser_t;       //deg

    //configuration parameters
    double m_beta_angle_threshold;
    double m_gain_lin;
    double m_gain_ang;
    double m_goal_tolerance_lin;  //m 
    double m_goal_tolerance_ang;  //deg
    double m_max_lin_speed;       //m/s
    double m_max_ang_speed;       //deg/s
    double m_min_lin_speed;       //m/s
    double m_min_ang_speed;       //deg/s
    double m_default_beta_angle_threshold;
    double m_default_gain_lin;
    double m_default_gain_ang;
    double m_default_goal_tolerance_lin;  //m 
    double m_default_goal_tolerance_ang;  //deg
    double m_default_max_lin_speed;       //m/s
    double m_default_max_ang_speed;       //deg/s
    double m_default_min_lin_speed;       //m/s
    double m_default_min_ang_speed;       //deg/s

    int    m_loc_timeout_counter;
    int    m_las_timeout_counter;

    //semaphore
    Semaphore m_mutex;

protected:
    //pause info
    double m_pause_start;
    double m_pause_duration;

    //ports
    PolyDriver                      m_ptf;
    PolyDriver                      m_pLas;
    PolyDriver                      m_pLoc;
    IRangefinder2D*                 m_iLaser;
    ILocalization2D*                m_iLoc;
    BufferedPort<yarp::sig::Vector> m_port_target_input;
    BufferedPort<yarp::os::Bottle>  m_port_commands_output;
    BufferedPort<yarp::os::Bottle>  m_port_status_output;
    BufferedPort<yarp::os::Bottle>  m_port_speak_output;
    BufferedPort<yarp::os::Bottle>  m_port_gui_output;
    yarp::os::Node*                 m_rosNode;
    rosGoalSubscriber               m_rosGoalInputPort;
    rosGoalPublisher                m_rosCurrentGoal;
    rosPathPublisher                m_localPlan;
    rosPathPublisher                m_globalPlan;
    
    Property             m_robotCtrl_options;
    ResourceFinder       &m_rf;
    yarp::dev::Map2DLocation    m_localization_data;
    target_type          m_target_data;
    std::vector<LaserMeasurementData>  m_laser_data;
    struct
    {
       double linear_vel;
       double linear_dir;
       double angular_vel;
       void zero() { linear_vel = 0; linear_dir = 0; angular_vel = 0; }
    }
    m_control_out;
    NavigationStatusEnum m_status;
    int                  m_retreat_counter;
    bool                 m_useGoalFromRosTopic;
    bool                 m_publishRosStuff;
    string               m_frame_robot_id;
    string               m_frame_map_id;
    double               m_min_laser_angle;
    double               m_max_laser_angle;
    double               m_laser_angle_of_view;
    double               m_time_of_obstacle_detection;
    double               m_time_ob_obstacle_removal;

    obstacles_class*     m_obstacle_handler;

    
    
    ////////////////////////////////////////
    //METHOD
    ///////////////////////////////////////
public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

    void          getCurrentPos(yarp::dev::Map2DLocation& loc);
    string        getMapId();
    void          setNewAbsTarget(yarp::sig::Vector target);
    void          setNewRelTarget(yarp::sig::Vector target);
    void          resetParamsToDefaultValue();
    void          stopMovement();
    void          pauseMovement (double secs);
    void          resumeMovement();
    string        getNavigationStatusAsString();
    int           getNavigationStatusAsInt();
    Map2DLocation getCurrentAbsTarget();
    Map2DLocation getCurrentRelTarget();
    void          printStats();
    
private:
    bool        rosInit(const yarp::os::Bottle& ros_group);
    void        sendOutput();
    bool inline evaluateLocalization();
    void inline evaluateGoalFromTopic();
    void inline getLaserData();
    void inline sendCurrentGoal();
    void inline publishLocalPlan();
    void inline saturateRobotControls();

};

#endif
