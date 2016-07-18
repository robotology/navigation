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
#include <string>
#include <math.h>

#include "status.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define TIMEOUT_MAX 300
const double RAD2DEG  = 180.0/M_PI;
const double DEG2RAD  = M_PI/180.0;

struct target_type
{
    yarp::sig::Vector target;
    bool              weak_angle;

    target_type() {weak_angle=false; target.resize(3,0.0);}
    double& operator[] (const int& i) { return target[i]; }
};

class laser_type
{
    yarp::os::Mutex  m_mutex;
    unsigned int     m_data_size;
    double           m_laser_angle_of_view;
    double* m_laser_x;
    double* m_laser_y;
    double* m_distances;
    double* m_angles;
    double  m_laser_pos_x;
    double  m_laser_pos_y;
    double  m_laser_pos_t;
    
    public:
    laser_type(size_t data_size, double laser_angle_of_view)
    {
        m_data_size = data_size;
        m_laser_angle_of_view = laser_angle_of_view;
        unsigned int i = 0;
        m_laser_x = new double[m_data_size];
        m_laser_y = new double[m_data_size];
        m_distances = new double[m_data_size];
        m_angles = new double[m_data_size];
        for (i = 0; i<data_size; i++) m_laser_x[i] = 0.0;
        for (i = 0; i<data_size; i++) m_laser_y[i] = 0.0;
        for (i = 0; i<data_size; i++) m_distances[i] = 0.0;
        for (i = 0; i<data_size; i++) m_angles[i] = 0.0;
        m_laser_pos_x = 0;
        m_laser_pos_y = 0;
        m_laser_pos_t = 0;
    }

    ~laser_type()
    {
        if (m_laser_x)   { delete [] m_laser_x; m_laser_x = 0; }
        if (m_laser_y)   { delete [] m_laser_y; m_laser_y = 0; }
        if (m_distances) { delete [] m_distances; m_distances = 0; }
        if (m_angles)    { delete [] m_angles; m_angles = 0; }
    }

    size_t size()
    {
        yarp::os::LockGuard lock(m_mutex);
        return m_data_size;
    }

    void set_cartesian_laser_data (const yarp::sig::Vector laser_map)
    {
        yarp::os::LockGuard lock (m_mutex);
        size_t scansize = laser_map.size();
        for (unsigned int i = 0; i<scansize; i++)
        {
            double angle = (i / double(scansize) * m_laser_angle_of_view - m_laser_pos_t)* DEG2RAD;
            m_laser_x[i] = laser_map[i] * cos(angle) + m_laser_pos_x;
            m_laser_y[i] = laser_map[i] * sin(angle) + m_laser_pos_y;
            m_distances[i] = sqrt(m_laser_x[i] * m_laser_x[i] + m_laser_y[i] * m_laser_y[i]);
            m_angles[i] = atan2(double(m_laser_x[i]), double(m_laser_y[i]))*RAD2DEG;
        }
    }

    inline const double& get_distance(int i) { yarp::os::LockGuard lock(m_mutex); return m_distances[i]; }
    inline const double& get_angle(int i) { yarp::os::LockGuard lock(m_mutex); return m_angles[i]; }
    inline const double& get_x(int i) { yarp::os::LockGuard lock(m_mutex); return m_laser_x[i]; }
    inline const double& get_y(int i) { yarp::os::LockGuard lock(m_mutex); return m_laser_y[i]; }
    inline void set_laser_position(double x, double y, double t) { yarp::os::LockGuard lock(m_mutex); m_laser_pos_x = x; m_laser_pos_y = y; m_laser_pos_t = t; }
};

class GotoThread: public yarp::os::RateThread
{
    private:
    void sendOutput();

    public:
    bool   enable_retreat;
    double goal_tolerance_lin;  //m 
    double goal_tolerance_ang;  //deg

    public:
    //configuration parameters
    double k_ang_gain;
    double k_lin_gain;
    double max_lin_speed;       //m/s
    double max_ang_speed;       //deg/s
    double min_lin_speed;       //m/s
    double min_ang_speed;       //deg/s
    double robot_radius;        //m
    double robot_laser_x;       //m
    double robot_laser_y;       //m
    double robot_laser_t;       //deg
    int    retreat_duration; 

    int    loc_timeout_counter;
    int    odm_timeout_counter;
    int    las_timeout_counter;

    //semaphore
    Semaphore mutex;

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
    
    Property            robotCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   odometry_data;
    target_type         target_data;
    laser_type*         laser_data;
    yarp::sig::Vector   control_out;
    status_type         status;
    int                 retreat_counter;
    bool                use_odometry;
    bool                use_localization_from_port;
    bool                use_localization_from_tf;
    string              frame_robot_id;
    string              frame_map_id;
    double              min_laser_angle;
    double              max_laser_angle;
    double              laser_angle_of_view;

    //obstacles_emergency_stop block
    public:
    bool                enable_obstacles_emergency_stop;
    bool                enable_dynamic_max_distance;
    double              obstacle_time;
    double              max_obstacle_wating_time;
    double              safety_coeff;
    double              max_detection_distance;
    double              min_detection_distance;
    double              obstacle_removal_time;

    //obstacle avoidance block
    public:
    bool                enable_obstacles_avoidance;
    double              max_obstacle_distance;
    double              frontal_blind_angle;
    double              speed_reduction_factor;
    double              angle_f;
    double              angle_t;
    double              angle_g;
    double              w_f;
    double              w_t;
    double              w_g;

    public:
    GotoThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               robotCtrl_options (options)
    {
        status = IDLE;
        loc_timeout_counter = TIMEOUT_MAX;
        odm_timeout_counter = TIMEOUT_MAX;
        las_timeout_counter = TIMEOUT_MAX;
        localization_data.resize(3,0.0);
        retreat_counter = 0;
        safety_coeff = 1.0;
        enable_obstacles_emergency_stop = false;
        enable_obstacles_avoidance      = false;
        enable_dynamic_max_distance     = false;
        enable_retreat                  = false;
        retreat_duration                = 300;
        control_out.resize(3,0.0);
        pause_start = 0;
        pause_duration = 0;
        goal_tolerance_lin = 0.05;
        goal_tolerance_ang = 0.6;
        max_obstacle_wating_time = 60.0;
        max_obstacle_distance = 0.8;
        frontal_blind_angle = 25.0;
        speed_reduction_factor = 0.70;
        max_detection_distance = 1.5;
        min_detection_distance = 0.4;
        obstacle_removal_time = 0.0;
        iLaser = 0;
        iTf = 0;
        min_laser_angle = 0;
        max_laser_angle = 0;
        robot_radius = 0;
        robot_laser_x = 0;
        robot_laser_y = 0;
        robot_laser_t = 0;
        laser_data = 0;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        k_ang_gain = 0.05;
        k_lin_gain = 0.1;
        max_lin_speed = 0.9;  //m/s
        max_ang_speed = 10.0; //deg/s
        min_lin_speed = 0.0;  //m/s
        min_ang_speed = 0.0; //deg/s
        use_odometry = true;
        use_localization_from_port = false;
        use_localization_from_tf = false;
        yInfo ("Using following paramters: %s", rf.toString().c_str());
        if (rf.check("ang_speed_gain"))     {k_ang_gain = rf.find("ang_speed_gain").asDouble();}
        if (rf.check("lin_speed_gain"))     {k_lin_gain = rf.find("lin_speed_gain").asDouble();}
        if (rf.check("max_lin_speed"))      {max_lin_speed = rf.find("max_lin_speed").asDouble();}
        if (rf.check("max_ang_speed"))      {max_ang_speed = rf.find("max_ang_speed").asDouble();}
        if (rf.check("min_lin_speed"))      {min_lin_speed = rf.find("min_lin_speed").asDouble();}
        if (rf.check("min_ang_speed"))      {min_ang_speed = rf.find("min_ang_speed").asDouble();}
        
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

        bool ff = geometry_group.check("robot_radius");
        ff &= geometry_group.check("laser_pos_x");
        ff &= geometry_group.check("laser_pos_y");
        ff &= geometry_group.check("laser_pos_theta");

        if (ff)
        {
            robot_radius = geometry_group.find("robot_radius").asDouble();
            robot_laser_x = geometry_group.find("laser_pos_x").asDouble();
            robot_laser_y = geometry_group.find("laser_pos_y").asDouble();
            robot_laser_t = geometry_group.find("laser_pos_theta").asDouble();
        }
        else
        {
            yError() << "Invalid/missing parameter in ROBOT_GEOMETRY group";
            return false;
        }

        if (rf.check("goal_tolerance_lin")) {goal_tolerance_lin = rf.find("goal_tolerance_lin").asDouble();}
        if (rf.check("goal_tolerance_ang")) {goal_tolerance_ang = rf.find("goal_tolerance_ang").asDouble();}
        if (localization_group.check("use_odometry"))       { use_odometry = (localization_group.find("use_odometry").asInt() == 1); }
        if (localization_group.check("use_localization_from_port")) { use_localization_from_port = (localization_group.find("use_localization_from_port").asInt() == 1); }
        if (localization_group.check("use_localization_from_tf"))   { use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }
        if (localization_group.check("robot_frame_id"))             { this->frame_robot_id = localization_group.find("robot_frame_id").asString(); }
        if (localization_group.check("map_frame_id"))               { this->frame_map_id = localization_group.find("map_frame_id").asString(); }
        if (use_localization_from_port == true && use_localization_from_tf == true)
        {
            yError() << "`use_localization_from_tf` and `use_localization_from_port` cannot be true simulteneously!";
            return false;
        }

        Bottle btmp;
        btmp = rf.findGroup("RETREAT_OPTION");
        if (btmp.check("enable_retreat",Value(0)).asInt()==1)
            enable_retreat = true;
        retreat_duration = btmp.check("retreat_duration",Value(300)).asInt();

        btmp = rf.findGroup("OBSTACLES_EMERGENCY_STOP");
        if (btmp.check("enable_obstacles_emergency_stop",Value(0)).asInt()==1)
            enable_obstacles_emergency_stop = true;
        if (btmp.check("enable_dynamic_max_distance",Value(0)).asInt()==1)
            enable_dynamic_max_distance = true;
        max_obstacle_wating_time = btmp.check("max_wating_time",Value(60.0)).asDouble();
        max_detection_distance   = btmp.check("max_detection_distance",Value(1.5)).asDouble();
        min_detection_distance   = btmp.check("min_detection_distance",Value(0.4)).asDouble();
        
        btmp = rf.findGroup("OBSTACLES_AVOIDANCE");
        if (btmp.check("enable_obstacles_avoidance",Value(0)).asInt()==1)
            enable_obstacles_avoidance = true;
        if (btmp.check("frontal_blind_angle"))
            frontal_blind_angle = btmp.check("frontal_blind_angle",Value(25.0)).asDouble();
        if (btmp.check("speed_reduction_factor"))
            speed_reduction_factor = btmp.check("speed_reduction_factor",Value(0.70)).asDouble();

        enable_retreat = false;
        retreat_duration = 300;

        //open module ports
        string localName = "/robotGoto";
        port_commands_output.open((localName+"/control:o").c_str());
        port_status_output.open((localName+"/status:o").c_str());
        port_odometry_input.open((localName+"/odometry:i").c_str());
        port_speak_output.open((localName+"/speak:o").c_str());
        port_gui_output.open((localName+"/gui:o").c_str());

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
        yarp::os::Network::connect("/robot/laser:o","/yarpLaserScannerGui/laser:i");
        yarp::os::Network::connect("/robotGoto/gui:o","/yarpLaserScannerGui/nav_display:i");

        //automatic port connections
        /*
        b = Network::connect((localName+"/commands:o").c_str(),"/robot/control:i", "udp", false);
        if (!b) {yError ("Unable to connect the output command port!"); return false;}
        */

        return true;
    }

    virtual void run();

    void setNewAbsTarget(yarp::sig::Vector target);
    void setNewRelTarget(yarp::sig::Vector target);
    void stopMovement();
    void pauseMovement (double secs);
    void resumeMovement();
    string getNavigationStatus();

    virtual void threadRelease()
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
    }

    void printStats();
    bool check_obstacles_in_path();
    bool compute_obstacle_avoidance();
    
    private:
    int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);

};

#endif
