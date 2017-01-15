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

#ifndef M_PI
#define M_PI 3.14159265
#endif

#ifndef X
#define X 0
#endif

#ifndef Y
#define Y 1
#endif

#ifndef ANGLE
#define ANGLE 2
#endif

#ifndef LIN_VEL
#define LIN_VEL 1
#endif

#ifndef ANG_VEL
#define ANG_VEL 2
#endif

#ifndef ANG_MOM
#define ANG_MOM 0
#endif

#define TIMEOUT_MAX 300
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

struct target_type
{
    yarp::sig::Vector target;
    bool              weak_angle;

    target_type() {weak_angle = false; target.resize(3,0.0);}
    double& operator[] (const int& i) { return target[i]; }
};

class GotoThread: public yarp::os::RateThread
{
    /////////////////////////////////////
    //PROPERTIES
    ////////////////////////////////////
public:
    bool   enable_retreat;
    int    retreat_duration;

    //robot properties
    double robot_is_holonomic;
    double robot_radius;        //m
    double robot_laser_x;       //m
    double robot_laser_y;       //m
    double robot_laser_t;       //deg

    //configuration parameters
    double m_max_gamma_angle;
    double m_gain_lin;
    double m_gain_ang;
    double m_goal_tolerance_lin;  //m 
    double m_goal_tolerance_ang;  //deg
    double m_max_lin_speed;       //m/s
    double m_max_ang_speed;       //deg/s
    double m_min_lin_speed;       //m/s
    double m_min_ang_speed;       //deg/s
    double m_default_max_gamma_angle;
    double m_default_gain_lin;
    double m_default_gain_ang;
    double m_default_goal_tolerance_lin;  //m 
    double m_default_goal_tolerance_ang;  //deg
    double m_default_max_lin_speed;       //m/s
    double m_default_max_ang_speed;       //deg/s
    double m_default_min_lin_speed;       //m/s
    double m_default_min_ang_speed;       //deg/s

    int    loc_timeout_counter;
    int    odm_timeout_counter;
    int    las_timeout_counter;

    //semaphore
    Semaphore mutex;

    //obstacles_emergency_stop block
    bool                 enable_obstacles_emergency_stop;
    bool                 enable_dynamic_max_distance;
    double               obstacle_time;
    double               max_obstacle_wating_time;
    double               safety_coeff;
    double               max_detection_distance;
    double               min_detection_distance;
    double               obstacle_removal_time;

    //obstacle avoidance block
    bool                 enable_obstacles_avoidance;
    double               max_obstacle_distance;
    double               frontal_blind_angle;
    double               speed_reduction_factor;
    double               angle_f;
    double               angle_t;
    double               angle_g;
    double               w_f;
    double               w_t;
    double               w_g;

protected:
    //pause info
    double pause_start;
    double pause_duration;

    //ports
    PolyDriver                      ptf;
    PolyDriver                      pLas;
    IRangefinder2D*                 iLaser;
    IFrameTransform*                iTf;
    BufferedPort<yarp::sig::Vector> port_odometry_input;
    BufferedPort<yarp::sig::Vector> port_localization_input;
    BufferedPort<yarp::sig::Vector> port_target_input;
    BufferedPort<yarp::os::Bottle>  port_commands_output;
    BufferedPort<yarp::os::Bottle>  port_status_output;
    BufferedPort<yarp::os::Bottle>  port_speak_output;
    BufferedPort<yarp::os::Bottle>  port_gui_output;
    yarp::os::Node*                 rosNode;
    rosGoalSubscriber               rosGoalInputPort;
    rosGoalPublisher                rosCurrentGoal;
    rosPathPublisher                localPlan;
    rosPathPublisher                globalPlan;
    
    Property             robotCtrl_options;
    ResourceFinder       &rf;
    yarp::sig::Vector    localization_data;
    yarp::sig::Vector    odometry_data;
    target_type          target_data;
    std::vector<LaserMeasurementData>  laser_data;
    yarp::sig::Vector    control_out;
    NavigationStatusEnum status;
    int                  retreat_counter;
    bool                 m_use_odometry;
    bool                 m_use_localization_from_port;
    bool                 m_use_localization_from_tf;
    bool                 m_useGoalFromRosTopic;
    bool                 m_publishRosStuff;
    string               frame_robot_id;
    string               frame_map_id;
    double               min_laser_angle;
    double               max_laser_angle;
    double               laser_angle_of_view;

    
    
    ////////////////////////////////////////
    //METHOD
    ///////////////////////////////////////
public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();

    void          getCurrentPos(yarp::sig::Vector& v);
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
    bool          check_obstacles_in_path();
    bool          compute_obstacle_avoidance();
    
private:
    bool        rosInit(const yarp::os::Bottle& ros_group);
    int         pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
    void        sendOutput();
    void inline evaluateLocalization();
    void inline evaluateGoalFromTopic();
    void inline getLaserData();
    void inline sendCurrentGoal();
    void inline publishLocalPlan();

};

#endif
