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
#include <yarp/dev/IRangefinder2d.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Semaphore.h>
#include <string>

#include "status.h"
#include "map.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define TIMEOUT_MAX 100

class PlannerThread: public yarp::os::RateThread
{
    public:
    double goal_tolerance_lin;       //m 
    double goal_tolerance_ang;       //deg
    double waypoint_tolerance_lin;   //m 
    double waypoint_tolerance_ang;   //deg
    double max_lin_speed;            //m/s
    double min_lin_speed;            //m/s
    double max_ang_speed;            //deg/s
    double min_ang_speed;            //deg/s
    int    min_waypoint_distance;    //cells

    //semaphore
    Semaphore mutex;

    protected:
    //configuration parameters
    double    robot_radius;     //m
    map_class map;
    bool      use_optimized_path;
    double    min_laser_angle;
    double    max_laser_angle;

    //ports
    BufferedPort<yarp::sig::Vector>                        port_localization_input;
    BufferedPort<yarp::os::Bottle>                         port_status_input;
    IRangefinder2D*                                        iLaser;
    BufferedPort<yarp::os::Bottle>                         port_yarpview_target_input;
    BufferedPort<yarp::os::Bottle>                         port_yarpview_target_output;

    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > port_map_output;
    BufferedPort<yarp::os::Bottle>                         port_status_output;
    RpcClient                                              port_commands_output;

    Property            robotCtrl_options;
    ResourceFinder      &rf;
    yarp::sig::Vector   localization_data;
    yarp::sig::Vector   goal_data;
    struct lasermap_type
    {
        double x;
        double y;
        lasermap_type() {x=y=0.0;}
    };
    lasermap_type       laser_data[1080];
    std::vector<cell>   laser_map_cell;
    cell                current_waypoint;

    std::queue<cell>    computed_path;
    std::queue<cell>    computed_simplified_path;
    std::queue<cell>*   current_path;
    status_type         planner_status;
    status_type         inner_status;

    //timeouts
    public:
    int                 loc_timeout_counter;
    int                 laser_timeout_counter;
    int                 inner_status_timeout_counter;

    public:
    PlannerThread(unsigned int _period, ResourceFinder &_rf, Property options) :
               RateThread(_period),     rf(_rf),
               robotCtrl_options (options)
    {
        planner_status = IDLE;
        inner_status = IDLE;
        localization_data.resize(3,0.0);
        loc_timeout_counter = 0;
        laser_timeout_counter = 0;
        inner_status_timeout_counter = 0;
        goal_tolerance_lin = 0.05;
        goal_tolerance_ang = 0.6;
        waypoint_tolerance_lin = 0.05;
        waypoint_tolerance_ang = 0.6;
        max_lin_speed = 0.9;
        max_ang_speed = 10.0;
        min_lin_speed = 0.0;
        min_ang_speed = 0.0;
        use_optimized_path = true;
        current_path=&computed_simplified_path;
        min_waypoint_distance = 0;
        iLaser = 0;
        min_laser_angle = 0;
        max_laser_angle = 0;
    }

    virtual bool threadInit()
    {
        //read configuration parametes
        yarp::os::ResourceFinder mapFinder;
        mapFinder.setDefaultContext("robot/maps");
        mapFinder.configure(0,0);

        string map_filename;
        map_filename = mapFinder.getHomeContextPath().c_str() + string("/");
        map_filename = map_filename + rf.find("map_file").asString().c_str();
        //map_filename = rf.find("map_file").asString().c_str();
        if (!map.loadMap(map_filename))
        {
            yError("map file not found, closing");
            return false;
        }

        if (rf.check("waypoint_tolerance_lin")) {waypoint_tolerance_lin = rf.find("waypoint_tolerance_lin").asDouble();}
        if (rf.check("waypoint_tolerance_ang")) {waypoint_tolerance_ang = rf.find("waypoint_tolerance_ang").asDouble();}
        if (rf.check("goal_tolerance_lin"))     {goal_tolerance_lin = rf.find("goal_tolerance_lin").asDouble();}
        if (rf.check("goal_tolerance_ang"))     {goal_tolerance_ang = rf.find("goal_tolerance_ang").asDouble();}
        if (rf.check("use_optimized_path"))     {int p=rf.find("use_optimized_path").asInt(); select_optimized_path(p==1);}
        if (rf.check("max_lin_speed"))          {max_lin_speed = rf.find("max_lin_speed").asDouble();}
        if (rf.check("max_ang_speed"))          {max_ang_speed = rf.find("max_ang_speed").asDouble();}
        if (rf.check("min_lin_speed"))          {min_lin_speed = rf.find("min_lin_speed").asDouble();}
        if (rf.check("min_ang_speed"))          {min_ang_speed = rf.find("min_ang_speed").asDouble();}
        if (rf.check("min_waypoint_distance"))  {min_waypoint_distance = rf.find("min_waypoint_distance").asInt();}

        //open module ports
        string localName = "/robotPathPlanner";
        port_localization_input.open((localName+"/localization:i").c_str());
        port_status_input.open((localName+"/navigationStatus:i").c_str());
        port_status_output.open((localName+"/plannerStatus:o").c_str());
        port_commands_output.open((localName+"/commands:o").c_str());
        port_map_output.open((localName+"/map:o").c_str());
        port_yarpview_target_input.open((localName+"/yarpviewTarget:i").c_str());
        port_yarpview_target_output.open((localName+"/yarpviewTarget:o").c_str());

        PolyDriver p;
        Property options;
        options.put("device", "Rangefinder2DClient");
        options.put("local", "/robotPathPlanner/laser:i");
        options.put("remote", "/laser");
        if (p.open(options) == false)
        {
            yError() << "Unable to open laser driver";
            return false;
        }
        p.view(iLaser);
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

        //automatic port connections
        //bool b = false;
        //b = Network::connect("/robot_ros_bridge/localization:o",(localName+"/localization:i").c_str(), "udp", false);
        // if (!b) {yError ("Unable to connect the localization port!\n"); return false;}
        return true;
    }

    virtual void run();

    void setNewAbsTarget(yarp::sig::Vector target);
    void setNewRelTarget(yarp::sig::Vector target);
    void sendWaypoint();
    void startNewPath(cell target);
    void stopMovement();
    void pauseMovement (double secs);
    void resumeMovement();
    string getNavigationStatus();
    void select_optimized_path(bool b);

    virtual void threadRelease()
    {    
        port_localization_input.interrupt();
        port_localization_input.close();
        port_map_output.interrupt();
        port_map_output.close();
        port_status_input.interrupt();
        port_status_input.close();
        port_status_output.interrupt();
        port_status_output.close();
        port_commands_output.interrupt();
        port_commands_output.close();
        port_yarpview_target_input.interrupt();
        port_yarpview_target_input.close();
        port_yarpview_target_output.interrupt();
        port_yarpview_target_output.close();
    }

    void printStats();

};

#endif
