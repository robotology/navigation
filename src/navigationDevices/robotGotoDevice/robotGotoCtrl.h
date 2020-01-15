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
#include <yarp/dev/IFrameTransform.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/ILocalization2D.h>
#include <string>
#include <math.h>
#include <mutex>
#include <yarp/rosmsg/visualization_msgs/MarkerArray.h>
#include <yarp/rosmsg/geometry_msgs/PoseStamped.h>
#include <yarp/rosmsg/nav_msgs/Path.h>
#include <obstacles.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

typedef yarp::os::Subscriber<yarp::rosmsg::geometry_msgs::PoseStamped> rosGoalSubscriber;
typedef yarp::os::Publisher<yarp::rosmsg::geometry_msgs::PoseStamped>  rosGoalPublisher;
typedef yarp::os::Publisher<yarp::rosmsg::nav_msgs::Path>              rosPathPublisher;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define TIMEOUT_MAX 300
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

struct target_type
{
    yarp::dev::Nav2D::Map2DLocation target;
    bool                            weak_angle;

    target_type() {weak_angle = false;}
};

class GotoThread: public yarp::os::PeriodicThread
{
    /////////////////////////////////////
    //PROPERTIES
    ////////////////////////////////////
public:
    bool   m_enable_retreat;
    double m_retreat_duration_default;

    //robot properties
    bool   m_robot_is_holonomic;
    bool   m_enable_obstacles_emergency_stop;
    bool   m_enable_obstacles_avoidance;
    double m_robot_radius;        //m
    double m_robot_laser_x;       //m
    double m_robot_laser_y;       //m
    double m_robot_laser_t;       //deg

    //configuration parameters
    string m_local_name; 
    double m_beta_angle_threshold;
    double m_gain_lin;
    double m_gain_ang;
    double m_goal_tolerance_lin;  //m 
    double m_goal_tolerance_ang;  //deg
    double m_max_lin_speed;       //m/s
    double m_max_ang_speed;       //deg/s
    double m_min_lin_speed;       //m/s
    double m_min_ang_speed;       //deg/s
    double m_approach_direction;
    double m_approach_speed;
    double m_default_beta_angle_threshold;
    double m_default_gain_lin;
    double m_default_gain_ang;
    double m_default_goal_tolerance_lin;  //m 
    double m_default_goal_tolerance_ang;  //deg
    double m_default_max_lin_speed;       //m/s
    double m_default_max_ang_speed;       //deg/s
    double m_default_min_lin_speed;       //m/s
    double m_default_min_ang_speed;       //deg/s
    double m_default_approach_direction;
    double m_default_approach_speed;

    //watchdogs for data received from external sources
    double m_stats_time_last;
    double m_stats_time_curr;
    int    m_loc_timeout_counter;
    int    m_las_timeout_counter;

    //semaphore
    Semaphore m_mutex;

protected:
    //pause info
    double m_pause_start;
    double m_pause_duration;

    //yarp device drivers and interfaces
    PolyDriver                      m_ptf;
    PolyDriver                      m_pLas;
    PolyDriver                      m_pLoc;
    IRangefinder2D*                 m_iLaser;
    ILocalization2D*                m_iLoc;

    //yarp ports
    BufferedPort<yarp::sig::Vector> m_port_target_input;
    BufferedPort<yarp::os::Bottle>  m_port_commands_output;
    BufferedPort<yarp::os::Bottle>  m_port_status_output;
    BufferedPort<yarp::os::Bottle>  m_port_speak_output;
    BufferedPort<yarp::os::Bottle>  m_port_gui_output;

    //ROS topics
    yarp::os::Node*                 m_rosNode;
    rosGoalSubscriber               m_rosGoalInputPort;
    rosGoalPublisher                m_rosCurrentGoal;
    rosPathPublisher                m_localPlan;
    rosPathPublisher                m_globalPlan;
    
    Property                           m_robotCtrl_options;
    Searchable                         &m_cfg;
    yarp::dev::Nav2D::Map2DLocation    m_localization_data;
    target_type                        m_target_data;
    std::vector<LaserMeasurementData>  m_laser_data;
    
    NavigationStatusEnum m_status;
    NavigationStatusEnum m_status_after_approach;
    double               m_retreat_duration_time;
    double               m_retreat_starting_time;
    bool                 m_useGoalFromRosTopic;
    bool                 m_publishRosStuff;
    string               m_frame_robot_id;
    string               m_frame_map_id;
    double               m_min_laser_angle;
    double               m_max_laser_angle;
    double               m_laser_angle_of_view;
    double               m_time_of_obstacle_detection;
    double               m_time_ob_obstacle_removal;

    //obstacle handler
    obstacles_class*     m_obstacle_handler;

    //internal type definition to store control output
    struct
    {
       double linear_vel;
       double linear_dir;
       double angular_vel;
       void zero() { linear_vel = 0; linear_dir = 0; angular_vel = 0; }
    }
    m_control_out;
    
    ////////////////////////////////////////
    //METHODS
    ///////////////////////////////////////
public:
    //methods inherited from yarp::os::RateThread
    virtual bool threadInit() override;
    virtual void run() override;
    virtual void threadRelease() override;

    /**
    * Constructor.
    * @param _period the control loop period (default 0.020s)
    * @param _rf the resource finder containing the configuration options (from .ini file)
    * @param options additional configuration options
    */
    GotoThread(double _period, yarp::os::Searchable& _cfg);

    /**
    * Returns robot current position
    * @param loc the current absolute position of the robot
    * @return true if the command is executed successfully, false otherwise
    */
    bool          getCurrentPos(yarp::dev::Nav2D::Map2DLocation& loc);
    
    /**
    * Sets a new target, expressed in the map reference frame.
    * @param target a three-elements vector containing the robot pose (x,y,theta)
    */
    void          setNewAbsTarget(yarp::sig::Vector target);
    
    /**
    * Sets a new target, expressed in the robot reference frame.
    * @param target a three-elements vector containing the robot pose (x,y,theta)
    */
    void          setNewRelTarget(yarp::sig::Vector target);
    
    /**
    * Performs an open-loop movement: the robot is commanded to move in the desired direction for 
    * a determined amount of time, regardless the presence of obstacle in the path.
    * @param dir the desired direction (in the robot reference frame)
    * @param speed the velocity of the movement, expressed in m/s
    * @param time the duration of the approach command, expressed in seconds
    */
    void          approachTarget(double dir, double speed, double time);
    
    /**
    * Restores all internal parameters (such as max/min velocities, goal tolerance etc) to the values
    * defined in the configuration files.
    */
    void          resetParamsToDefaultValue();
    
    /**
    * Terminates a previously started navigation task.
    * @return true if the operation was successful, false otherwise.
    */
    bool          stopMovement();
    
    /**
    * Pauses the robot navigation
    * @param sec the duration of the pause, expressed in seconds. A negative number means forever.
    * @return true if the operation was successful, false otherwise.
    */
    bool          pauseMovement (double secs=-1);
    
    /**
    * Resumes the robot navigation after a previous pauseMovement()
    * @return true if the operation was successful, false otherwise.
    */
    bool          resumeMovement();
    
    /**
    * Returns the current navigation status used by the internal finite-state machine
    * @return the internal navigation status, expressed as a string
    */
    string        getNavigationStatusAsString();
    
    /**
    * Returns the current navigation status, used by the internal finite-state machine
    * @return the internal navigation status, expressed as an enum
    */
    NavigationStatusEnum getNavigationStatusAsInt();
    
    /**
    * Retrieves the current target, expressed in absolute (map) reference frame
    * @param the current target (set by a setNewAbsTarget)
    * @return true if the returned target is valid, false otherwise
    */
    bool getCurrentAbsTarget(Nav2D::Map2DLocation& target);
    
    /**
    * Retrieves the current target, expressed in robot reference frame
    * @param the current target (set by a setNewRelTarget)
    * @return true if the returned target is valid, false otherwise
    */
    bool getCurrentRelTarget(Nav2D::Map2DLocation& target);
    
    /**
    * Prints stats about the internal status of the module
    */
    void          printStats();
    
private:
    /**
    * Initializes the ROS system, opening the ROS node and the requested ROS topics.
    * @param ros_group the configuration options defined in [ROS_GROUP]. Valid parameters are parameters are reported in module description.
    * @return true if the ROS system initialization was successful.
    */
    bool        rosInit(const yarp::os::Bottle& ros_group);
    
    /**
    * Sends Cartesian velocities commands to a yarp output port (typically connected to baseControl)
    */
    void        sendOutput();
    
    /**
    * Obtains current robot position through a ILocalization2D interface.
    * @return true if data is successfully retrieved from the localization server, false otherwise
    */
    bool  evaluateLocalization();
    
    /**
    * Receives a new target from a ROS topic and starts the navigation
    */
    void evaluateGoalFromTopic();
    
    /**
    * Obtains laser data through a IRangefinder2D interface.
    */
    void getLaserData();
    
    /**
    * Publishes the current goal on the dedicated ROS topic.
    */
    void publishCurrentGoal();
    
    /**
    * Publishes the current local plan on the dedicated ROS topic.
    */
    void publishLocalPlan();
    
    /**
    * Checks the computed control outputs and saturates them if necessary.
    */
    void saturateRobotControls();

};

#endif
