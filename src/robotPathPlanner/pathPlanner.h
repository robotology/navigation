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

#ifndef PLANNER_THREAD_H
#define PLANNER_THREAD_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/ILocalization2D.h>
#include <string>
#include <visualization_msgs_MarkerArray.h>

#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;

#define TIMEOUT_MAX 100

class PlannerThread: public yarp::os::RateThread
{
    protected:
    double m_goal_tolerance_lin;       //m 
    double m_goal_tolerance_ang;       //deg
    double m_waypoint_tolerance_lin;   //m 
    double m_waypoint_tolerance_ang;   //deg
    double m_max_lin_speed;            //m/s
    double m_min_lin_speed;            //m/s
    double m_max_ang_speed;            //deg/s
    double m_min_ang_speed;            //deg/s
    int    m_min_waypoint_distance;    //cells

    //semaphore
    public:
    Semaphore m_mutex;

    protected:
    //configuration parameters
    double    m_robot_radius;        //m
    double    m_robot_laser_x;       //m
    double    m_robot_laser_y;       //m
    double    m_robot_laser_t;       //deg
    //map_class map;
    yarp::dev::MapGrid2D m_current_map;
    bool      m_use_optimized_path;
    double    m_min_laser_angle;
    double    m_max_laser_angle;
    double    m_laser_angle_of_view;
    string    m_frame_robot_id;
    string    m_frame_map_id;
    double    m_imagemap_refresh_time;

    //ports
    PolyDriver                                             m_ptf;
    PolyDriver                                             m_pLoc;
    PolyDriver                                             m_pLas;
    PolyDriver                                             m_pMap;
    IRangefinder2D*                                        m_iLaser;
    IMap2D*                                                m_iMap;
    ILocalization2D*                                       m_iLoc;
    BufferedPort<yarp::os::Bottle>                         m_port_status_input;
    BufferedPort<yarp::os::Bottle>                         m_port_yarpview_target_input;
    BufferedPort<yarp::os::Bottle>                         m_port_yarpview_target_output;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_port_map_output;
    BufferedPort<yarp::os::Bottle>                         m_port_status_output;
    RpcClient                                              m_port_commands_output;

    ResourceFinder                         &m_rf;
    yarp::dev::Map2DLocation               m_localization_data;
    yarp::dev::Map2DLocation               m_final_goal;
    std::queue<yarp::dev::Map2DLocation>   m_sequence_of_goals;
    std::vector<Map2DLocation>             m_locations_list;

    std::vector<yarp::dev::MapGrid2D::XYCell>   m_laser_map_cells;

    std::queue<MapGrid2D::XYCell>                 m_computed_path;
    std::queue<MapGrid2D::XYCell>                 m_computed_simplified_path;
    std::queue<MapGrid2D::XYCell>*                m_current_path;
    NavigationStatusEnum   m_planner_status;
    NavigationStatusEnum   m_inner_status;

    //timeouts
    protected:
    int                 m_loc_timeout_counter;
    int                 m_laser_timeout_counter;
    int                 m_inner_status_timeout_counter;

    //drawing flags
    public:
    bool                m_enable_draw_all_locations;

    public:
    virtual void run();

    public:
    bool          setNewAbsTarget(yarp::dev::Map2DLocation target);
    bool          setNewRelTarget(yarp::sig::Vector target);
    void          stopMovement();
    void          pauseMovement(double secs);
    void          resumeMovement();
    void          getCurrentPos(Map2DLocation& v);
    Map2DLocation getCurrentAbsTarget();
    Map2DLocation getCurrentRelTarget();
    string        getCurrentMapId();
    Map2DLocation getFinalAbsTarget();
    Map2DLocation getFinalRelTarget();
    string        getFinalMapId();
    string        getNavigationStatusAsString();
    int           getNavigationStatusAsInt();
    void          getTimeouts(int& localiz, int& laser, int& inner_status);

    private:
    bool          startPath();
    void          sendWaypoint();
    void          readTargetFromYarpView();
    bool          readLocalizationData();
    void          readLaserData();
    bool          readInnerNavigationStatus();
    void          draw_map();
    bool          getCurrentWaypoint(yarp::dev::MapGrid2D::XYCell &c) const;

    public:
    PlannerThread(unsigned int _period, ResourceFinder &_rf);
    virtual bool threadInit();
    virtual void threadRelease();
    void printStats();

};

#endif
