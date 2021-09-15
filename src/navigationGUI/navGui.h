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
#include <yarp/dev/INavigation2D.h>
#include <limits>
#include <string>

#include "navigation_defines.h"
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

class NavGuiThread: public yarp::os::PeriodicThread
{
    //semaphore
    public:
    std::mutex m_guithread_mutex;

    protected:
    //configuration parameters: robot geometric properties
    double    m_robot_radius = 0;        //m
    double    m_robot_laser_x = 0;       //m
    double    m_robot_laser_y = 0;       //m
    double    m_robot_laser_t = 0;       //deg
    double    m_min_laser_angle = 0;
    double    m_max_laser_angle = 0;
    double    m_laser_angle_of_view = 0;
    double    m_imagemap_draw_and_send_period = 0.033; //s

    //yarp device drivers and interfaces
    PolyDriver                                             m_ptf;
    PolyDriver                                             m_pLoc;
    PolyDriver                                             m_pLas;
    PolyDriver                                             m_pMap;
    PolyDriver                                             m_pNav;
    IRangefinder2D*                                        m_iLaser = nullptr;
    IMap2D*                                                m_iMap = nullptr;
    ILocalization2D*                                       m_iLoc = nullptr;
    INavigation2D*                                         m_iNav = nullptr;

    //yarp ports
    std::string                                            m_name = "/navigationGui";

    std::string                                            m_localization_client_device_name = LOCALIZATION_CLIENT_DEVICE_DEFAULT;
    std::string                                            m_map_client_device_name = MAP_CLIENT_DEVICE_DEFAULT;
    std::string                                            m_laser_client_device_name = LIDAR_CLIENT_DEVICE_DEFAULT;
    std::string                                            m_navigation_client_device_name = NAVIGATION_CLIENT_DEVICE_DEFAULT;

    std::string                                            m_remote_localization_port_name = "/localizationServer";
    std::string                                            m_remote_map_port_name = "/mapServer";
    std::string                                            m_remote_laser_port_name = "/ikart/laser:o";
    std::string                                            m_remote_navigation_port_name = "/navigationServer";

    std::string                                            m_las_carrier;
    std::string                                            m_map_carrier;
    std::string                                            m_nav_carrier;
    std::string                                            m_loc_carrier;

    BufferedPort<yarp::os::Bottle>                         m_port_yarpview_target_input;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_port_map_output;

    //internal data
    ResourceFinder                         &m_rf;
    yarp::dev::Nav2D::Map2DLocation        m_localization_data;
    yarp::dev::Nav2D::Map2DLocation        m_curr_goal;
    yarp::dev::Nav2D::Map2DLocation        m_curr_waypoint;
    yarp::dev::Nav2D::Map2DPath            m_global_waypoints;
    yarp::dev::Nav2D::Map2DPath            m_local_waypoints;
    std::vector<Map2DLocation>             m_locations_list;
    std::vector<Map2DArea>                 m_areas_list;

    //storage for the environment map
    yarp::dev::Nav2D::MapGrid2D m_current_map;
    yarp::dev::Nav2D::MapGrid2D m_temporary_obstacles_map;
    std::vector<yarp::dev::Nav2D::XYCell>   m_laser_map_cells;

    //statuses of the internal finite-state machine
    NavigationStatusEnum   m_navigation_status;
    NavigationStatusEnum   m_previous_navigation_status;

    //estimated poses (pointcloud localization)
    std::vector<yarp::dev::Nav2D::Map2DLocation> m_estimated_poses;

    //timeout counters (watchdog on the communication with external modules)
    protected:
    int                 m_loc_timeout_counter = 0;
    int                 m_laser_timeout_counter = 0;
    int                 m_nav_status_timeout_counter = 0;
    
    //refresh periods
    double              m_period_update_laser_data = 0.3;
    double              m_period_update_enlarged_obstacles = 1.0;
    double              m_period_update_estimated_poses = 1.0;
    double              m_period_update_map_locations = 5.0;
    double              m_period_update_global_map = std::numeric_limits<double>::infinity();

    //drawing flags: enable/disable drawing of particular objects on the GUI
    public:
    int                 m_enable_estimated_particles = 50; //this sets a max number on the particles drawn
    bool                m_enable_draw_all_locations = true;
    bool                m_enable_draw_laser_scans = true;
    bool                m_enable_draw_enlarged_scans = true;
    bool                m_enable_draw_infos = true;
    bool                m_enable_draw_global_path = true;
    bool                m_enable_draw_local_path = true;
  
    //images to be displayed
    IplImage* i1_map = nullptr;
    IplImage* i2_map_menu = nullptr;
    IplImage* i3_map_menu_scan = nullptr;
    IplImage* i4_map_with_path = nullptr;

    //buttons
    size_t button1_l;
    size_t button1_r;
    size_t button1_t;
    size_t button1_b;
    size_t button2_l;
    size_t button2_r;
    size_t button2_t;
    size_t button2_b;
    size_t button3_l;
    size_t button3_r;
    size_t button3_t;
    size_t button3_b;
    size_t button4_l;
    size_t button4_r;
    size_t button4_t;
    size_t button4_b;

    enum
    {
      button_status_localize,
      button_status_goto
    } button3_status;

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
    * Returns info about the current status of the navigation thread.
    * In particular, it returns the number of timeouts occurred while communicating with external modules.
    * @param localiz the number of timeouts occurred while getting localization data
    * @param laser the number of timeouts occurred while getting laser data
    * @param inner_status the number of timeouts occurred while communicating with the local navigation module
    */
    void          getTimeouts(int& localiz, int& laser, int& inner_status);

    private:
    void          sendTargetFromYarpView();
    void          readTargetFromYarpView();
    bool          readLocalizationData();
    bool          readMaps();
    void          readLaserData();
    bool          readWaypointsAndGoal();
    bool          readNavigationStatus(bool& changed);
    void          draw_and_send();
    bool          updateLocations();
    bool          updateAreas();
    bool          click_in_menu(yarp::os::Bottle *gui_targ, yarp::math::Vec2D<int>& click_p);
    void          addMenu(CvFont& font);

    public:
    /**
    * Constructor.
    * @param _period the main thread period (typical value 0.020s)
    * @param _rf the resource finder containing the configuration options
    */
    NavGuiThread(double _period, ResourceFinder &_rf);
    virtual ~NavGuiThread();

    //methods inherited from yarp::os::RateThread
    virtual void run() override;
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    
    /**
    * Prints stats about the internal status of the module
    */
    void printStats();

};

#endif
