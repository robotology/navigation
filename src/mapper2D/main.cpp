/*
 * Copyright (C)2017  ICub Facility - Istituto Italiano di Tecnologia
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
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Publisher.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/IRangefinder2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>
#include <visualization_msgs_MarkerArray.h>
#include <geometry_msgs_PoseStamped.h>
#include <geometry_msgs_PoseWithCovarianceStamped.h>
#include <nav_msgs_OccupancyGrid.h>

#include <math.h>

#include <cv.h>
#include <highgui.h> 

using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

/**
 * \section mapper2D
 * A module which registers laser scans to a map.
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value      | Required     | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:------------------:|:-----------: |:-----------------------------------------------------------------:|:-----:|
 * | GENERAL        |  module_name   | string  | -              | localizationServer | Yes          | The name of the module use to open ports                          |       |
 * | GENERAL        |  enable_ros    | int     | 0/1            | -                  | Yes          | If set to 1, the module will open the ROS topic specified by ROS::initialpose_topic parameter     |       |
 * | MAP            |  connect_to_yarp_mapserver   | int  | 0/1     |   -              | Yes          | If set to 1, LocalizationServer will ask maps to yarp map server when initial pose is updated | -     |
 * | ROS            |  initialpose_topic   | string     | -         |   -              | No           | Name of the topic which will be used to publish the initial pose  | -     |
 * | ROS            |  occupancygrid_topic | string     | -         |   -              | No           | Name of the topic which will be used to publish map data when initial pose is updated         | -     |
 * | TF             |  map_frame_id      | string     | -         |   -                | Yes          | Name of the map reference frame                                   | e.g. /map    |
 * | TF             |  robot_frame_id    | string     | -         |   -                | Yes          | Name of the robot reference frame                                 | e.g. /mobile_base    |
 * | LOCALIZATION   |  use_localization_from_odometry_port    | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a port to receive localization data                         | Incompatible with 'use_localization_from_tf=1'  |
 * | LOCALIZATION   |  use_localization_from_tf      | int      | 0/1  |   -         | Yes          | If set to 1, the module will use a tfClient to receive localization data                     | Incompatible with 'use_localization_from_odometry_port=1 |
 * | ODOMETRY       |  odometry_broadcast_port       | string   |  -   |   -         | Yes          | Full name of port broadcasting the localization data. The server will connect to this port.  | -    |
 */

class mapperModule : public yarp::os::RFModule
{
protected:
    std::string                  m_module_name;
    bool                         m_ros_enabled;
    yarp::dev::MapGrid2D         m_current_map;
    yarp::dev::Map2DLocation     m_localization_data;
    yarp::dev::PolyDriver        m_ptf;
    yarp::dev::PolyDriver        m_pmap;
    yarp::os::ResourceFinder     m_rf;
    yarp::os::Port               m_rpcPort;
    yarp::os::Mutex              m_mutex;

    bool                         m_localization_requested;
    bool                         m_clear_robot_way_enabled;
    bool                         m_mapping_running;
    bool                         m_use_localization_from_odometry_port;
    bool                         m_use_localization_from_tf;
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::sig::Vector>  m_port_odometry_input;
    double                       m_last_odometry_data_received;
    yarp::dev::IFrameTransform*  m_iTf;
    yarp::dev::IMap2D*           m_iMap;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_map_id;
    double                       m_last_statistics_printed;

    //locations
    std::vector<yarp::dev::Map2DLocation>   m_locations_array;

    //map
    double                       m_map_width_in_meters;
    double                       m_map_height_in_meters;
    double                       m_map_resolution;
    BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > m_port_map_output;
    BufferedPort<yarp::os::Bottle> m_port_yarpview_target_input;

    //laser
    bool                         m_laser_requested;
    yarp::dev::PolyDriver        m_pLas;
    yarp::dev::IRangefinder2D*   m_iLaser;
    int                          m_laser_timeout_counter;
    double                       m_min_laser_angle;
    double                       m_max_laser_angle;
    double                       m_laser_angle_of_view;
    std::vector<yarp::dev::MapGrid2D::XYCell>   m_laser_map_cells_array;

    //Menu 
    bool                         m_map_changed;
    CvPoint                      m_menu_lu[4];
    CvPoint                      m_menu_rd[4];
    int                          m_pixel_size;
    yarp::dev::MapGrid2D::map_flags m_selected_flag;

    //ros
    double                            m_map_publish_period;
    yarp::os::Node*                   m_rosNode;
    std::string                       m_topic_occupancyGrid;
    yarp::os::Publisher<nav_msgs_OccupancyGrid> m_rosPublisher_occupancyGrid;

public:
    mapperModule()
    {
        m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_UNKNOWN;
        m_iMap = 0;
        m_iTf  = 0;
        m_iLaser =0;
        m_laser_timeout_counter =0 ;
        m_min_laser_angle = 0;
        m_max_laser_angle = 0;
        m_laser_angle_of_view =0;
        m_rosNode = 0;
        m_map_changed=false;
        m_ros_enabled = false;
        m_mapping_running = false;
        m_laser_requested = true;
        m_localization_requested = true;
        m_module_name = "mapper2D";
        m_tf_data_received = -1;
        m_last_odometry_data_received = -1;
        m_last_statistics_printed = -1;
        m_clear_robot_way_enabled = true;
        m_map_publish_period = 1.0;

        m_localization_data.map_id = "unknown";
        m_localization_data.x = nan("");
        m_localization_data.y = nan("");
        m_localization_data.theta = nan("");

        int button_width=60;
        int button_space=60;
        m_menu_lu[0] = cvPoint(100,50);
        m_menu_lu[1] = cvPoint(100+button_width*1+button_space*1,50);
        m_menu_lu[2] = cvPoint(100+button_width*2+button_space*2,50);
        m_menu_lu[3] = cvPoint(100+button_width*3+button_space*3,50);

        m_menu_rd[0] = cvPoint(100+button_width,50+button_width);
        m_menu_rd[1] = cvPoint(100+button_width+button_space*1+button_width*1,   50+button_width);
        m_menu_rd[2] = cvPoint(100+button_width+button_space*2+button_width*2,   50+button_width);
        m_menu_rd[3] = cvPoint(100+button_width+button_space*3+button_width*3,   50+button_width);
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        m_rpcPort.open("/"+m_module_name+"/rpc");
        attach(m_rpcPort);
        //attachTerminal();
        m_rf = rf;
        
        //configuration file cheking
        Bottle general_group = m_rf.findGroup("GENERAL");
        if (general_group.isNull())
        {
            yError() << "Missing GENERAL group!";
            return false;
        }

        Bottle localization_group = m_rf.findGroup("LOCALIZATION");
        if (localization_group.isNull())
        {
            yError() << "Missing LOCALIZATION group!";
            return false;
        }

        Bottle ros_group = m_rf.findGroup("ROS");

        Bottle tf_group = m_rf.findGroup("TF");
        if (tf_group.isNull())
        {
            yError() << "Missing TF group!";
            return false;
        }

        Bottle odometry_group = m_rf.findGroup("ODOMETRY");
        if (odometry_group.isNull())
        {
            yError() << "Missing ODOMETRY group!";
            return false;
        }

        Bottle map_group = m_rf.findGroup("MAP");
        if (map_group.isNull())
        {
            yError() << "Missing MAP group!";
            return false;
        }
        yDebug() << map_group.toString();

        //general group
        if (general_group.check("module_name") == false)
        {
            yError() << "Missing `module_name` in [GENERAL] group";
            return false;
        }
        m_module_name = general_group.find("module_name").asString();

        if (general_group.check("enable_ros") == false)
        {
            yError() << "Missing `ros_enable` in [GENERAL] group";
            return false;
        }
        m_ros_enabled = (general_group.find("enable_ros").asInt()==1);

        if (general_group.check("use_laser") == false)
        {
            yError() << "Missing `use_laser` in [GENERAL] group";
            return false;
        }
        m_laser_requested = (general_group.find("use_laser").asInt()==1);

        if (general_group.check("use_localization") == false)
        {
            yError() << "Missing `use_localization` in [GENERAL] group";
            return false;
        }
        m_localization_requested = (general_group.find("use_localization").asInt()==1);

        //ros group
        if (m_ros_enabled)
        {
            m_rosNode = new yarp::os::Node("/"+m_module_name);
                    
            if (ros_group.check ("occupancygrid_topic"))
            {
                m_topic_occupancyGrid = ros_group.find ("occupancygrid_topic").asString();
                if (!m_rosPublisher_occupancyGrid.topic(m_topic_occupancyGrid))
                {
                     if (m_rosNode)
                     {
                         delete m_rosNode;
                         m_rosNode=0;
                     }
                     yError() << "localizationModule: unable to publish data on " << m_topic_occupancyGrid << " topic, check your yarp-ROS network configuration";
                     return false;
                }
            }
            if (ros_group.check ("map_publish_period"))
            {
                m_map_publish_period = ros_group.find ("map_publish_period").asDouble();
            }
        }

        //localization group
        if (localization_group.check("use_localization_from_odometry_port")) { m_use_localization_from_odometry_port = (localization_group.find("use_localization_from_odometry_port").asInt() == 1); }
        if (localization_group.check("use_localization_from_tf"))   { m_use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }

        if (m_use_localization_from_odometry_port == true && m_use_localization_from_tf == true)
        {
            yError() << "`use_localization_from_tf` and `use_localization_from_odometry_port` cannot be true simulteneously!";
            return false;
        }

        //tf group
        if (tf_group.check("map_frame_id") == false)
        {
            yError() << "Missing `map_frame_id` in [TF] group";
            return false;
        }
        if (tf_group.check("robot_frame_id") == false)
        {
            yError() << "Missing `robot_frame_id` in [TF] group";
            return false;
        }
        m_frame_map_id = tf_group.find("map_frame_id").asString();
        m_frame_robot_id = tf_group.find("robot_frame_id").asString();

        //odometry group
        if (odometry_group.check("odometry_broadcast_port") == false)
        {
            yError() << "Missing `odometry_port` in [ODOMETRY] group";
            return false;
        }
        m_port_broadcast_odometry_name = odometry_group.find("odometry_broadcast_port").asString();

        //device driver opening and/or connections
        if (m_use_localization_from_odometry_port)
        {
            std::string odom_portname = "/" + m_module_name + "/odometry:i";
            bool b1 = m_port_odometry_input.open(odom_portname.c_str());
            bool b2 = yarp::os::Network::sync(odom_portname.c_str(),false);
            bool b3 = yarp::os::Network::connect(m_port_broadcast_odometry_name.c_str(), odom_portname.c_str());
            if (b1 == false || b2 ==false || b3==false)
            {
                yError() << "Unable to initialize odometry port connection from " << m_port_broadcast_odometry_name.c_str()<< "to:" << odom_portname.c_str();
                return false;
            }
        }

        if (m_use_localization_from_tf)
        {
            Property options;
            options.put("device", "transformClient");
            options.put("local", "/"+m_module_name + "/TfClient");
            options.put("remote", "/transformServer");
            if (m_ptf.open(options) == false)
            {
                yError() << "Unable to open transform client";
                return false;
            }
            m_ptf.view(m_iTf);
            if (m_ptf.isValid() == false || m_iTf == 0)
            {
                yError() << "Unable to view iTransform interface";
                return false;
            }
        }

        //init MapServer
        Property map_options;
        map_options.put("device", "map2DClient");
        map_options.put("local", "/" +m_module_name); //This is just a prefix. map2DClient will complete the port name.
        map_options.put("remote", "/mapServer");
        if (m_pmap.open(map_options) == false)
        {
            yError() << "Unable to open mapClient";
            return false;
        }
        else
        {
            yInfo() << "Opened mapClient";
            m_pmap.view(m_iMap);
            if (m_pmap.isValid() == false || m_iMap == 0)
            {
                yError() << "Unable to view map interface";
                return false;
            }
        }

        if (m_laser_requested && init_laser()==false)
        {
            return false;
        }

        if (init_map()==false)
        {
            return false;
        }

        //map image output port
        m_port_map_output.open(("/"+m_module_name+"/map:o").c_str());
        m_port_yarpview_target_input.open(("/"+m_module_name+"/yarpviewTarget:i").c_str());
        return true;
    }

   void update_cell(MapGrid2D::XYCell c_start, yarp::dev::MapGrid2D::map_flags flag)
   {
       m_current_map.setMapFlag(c_start,flag);
       if (flag == yarp::dev::MapGrid2D::MAP_CELL_FREE)
       {m_current_map.setOccupancyData(c_start,0);}
       if (flag == yarp::dev::MapGrid2D::MAP_CELL_WALL)
       {m_current_map.setOccupancyData(c_start,100);}
   }

   void readTargetFromYarpView()
   {
        yarp::os::Bottle *gui_targ = m_port_yarpview_target_input.read(false);
        if (gui_targ)
        {
            if (gui_targ->size() == 2)
            {
                MapGrid2D::XYCell c_start;
                c_start.x = (*gui_targ).get(0).asInt();
                c_start.y = (*gui_targ).get(1).asInt();

#if 0
                if (c_start.x> m_menu_lu[0].x && c_start.x<m_menu_rd[0].x &&
                    c_start.y> m_menu_lu[0].y && c_start.y<m_menu_rd[0].y)
                {
                    m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_KEEP_OUT;
                    yDebug() << "MAP_CELL_KEEP_OUT selected";
                }
                if (c_start.x> m_menu_lu[1].x && c_start.x<m_menu_rd[1].x &&
                    c_start.y> m_menu_lu[1].y && c_start.y<m_menu_rd[1].y)
                {
                    m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_FREE;
                    yDebug() << "MAP_CELL_FREE selected";
                }
                if (c_start.x> m_menu_lu[2].x && c_start.x<m_menu_rd[2].x &&
                    c_start.y> m_menu_lu[2].y && c_start.y<m_menu_rd[2].y)
                {
                    m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_WALL;
                    yDebug() << "MAP_CELL_WALL selected";
                }
                if (c_start.x> m_menu_lu[3].x && c_start.x<m_menu_rd[3].x &&
                    c_start.y> m_menu_lu[3].y && c_start.y<m_menu_rd[3].y)
                {
                    m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_UNKNOWN;
                    yDebug() << "MAP_CELL_UNKNOWN selected";
                }
#endif

                yarp::sig::Vector v = static_cast<yarp::sig::Vector>(m_current_map.cell2World(c_start));
                yInfo("selected point is located at (%6.3f, %6.3f)", v[0], v[1]);

                if  (m_pixel_size==2)
                {
                    update_cell(MapGrid2D::XYCell(c_start.x  ,c_start.y  ),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x+1,c_start.y  ),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x  ,c_start.y+1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x+1,c_start.y+1),m_selected_flag);
                }
                else if  (m_pixel_size==3)
                {
                    update_cell(MapGrid2D::XYCell(c_start.x  ,c_start.y  ),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x+1,c_start.y  ),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x-1,c_start.y  ),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x+1,c_start.y+1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x-1,c_start.y+1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x+1,c_start.y-1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x-1,c_start.y-1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x  ,c_start.y+1),m_selected_flag);
                    update_cell(MapGrid2D::XYCell(c_start.x  ,c_start.y-1),m_selected_flag);
                }
                else
                {
                    update_cell(c_start,m_selected_flag);
                }
                m_map_changed=true;
            }
        }
    }

    virtual bool interruptModule()
    {
        m_rpcPort.interrupt();
        m_port_map_output.interrupt();
        m_port_yarpview_target_input.interrupt();
        return true;
    }

    virtual bool close()
    {
        if (m_ptf.isValid())
        {
            m_ptf.close();
        }
        if (m_pmap.isValid())
        {
            m_pmap.close();
        }

        if (m_rosPublisher_occupancyGrid.asPort().isOpen())
        {
            m_rosPublisher_occupancyGrid.interrupt();
            m_rosPublisher_occupancyGrid.close();
        }
        if (m_rosNode)
        {
            delete m_rosNode;
            m_rosNode = 0;
        }
        m_rpcPort.interrupt();
        m_rpcPort.close();

        m_port_map_output.interrupt();
        m_port_map_output.close();

        m_port_yarpview_target_input.interrupt();
        m_port_yarpview_target_input.close();
        return true;
    }

    virtual double getPeriod()
    { 
        return 0.1; 
    }
    
    void printStats()
    {
        static int counter = 0;
        yInfo() << "Module running since " << counter << "s";
        counter+=10;
    }

    bool init_laser()
    {
        //open the laser interface
        Bottle laserBottle = m_rf.findGroup("LASER");
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
        std::string laser_remote_port = laserBottle.find("laser_port").asString();

        Property las_options;
        las_options.put("device", "Rangefinder2DClient");
        las_options.put("local", "/robotPathPlanner/laser:i");
        las_options.put("remote", laser_remote_port);
        las_options.put("period", "10");
        if (m_pLas.open(las_options) == false)
        {
            yError() << "Unable to open laser driver";
            return false;
        }
        m_pLas.view(m_iLaser);
        if (m_iLaser == 0)
        {
            yError() << "Unable to open laser interface";
            return false;
        }
        if (m_iLaser->getScanLimits(m_min_laser_angle, m_max_laser_angle) == false)
        {
            yError() << "Unable to obtain laser scan limits";
            return false;
        }
        return true;
    }

    void publish_map()
    {
        if (m_rosNode)
        {
            double tmp=0;
            nav_msgs_OccupancyGrid& ogrid = m_rosPublisher_occupancyGrid.prepare();
            ogrid.clear();
            ogrid.info.height=m_current_map.height();
            ogrid.info.width=m_current_map.width();
            m_current_map.getResolution(tmp);
            ogrid.info.resolution=tmp;
            ogrid.header.frame_id="map";
            ogrid.info.map_load_time.sec=0;
            ogrid.info.map_load_time.nsec=0;
            double x, y, t;
            m_current_map.getOrigin(x,y,t);
            ogrid.info.origin.position.x=x;
            ogrid.info.origin.position.y=y;
            yarp::math::Quaternion q;
            yarp::sig::Vector v(4);
            v[0]=0; v[1]=0; v[2]=1; v[3]=t*DEG2RAD;
            q.fromAxisAngle(v);
            ogrid.info.origin.orientation.x = q.x();
            ogrid.info.origin.orientation.y = q.y();
            ogrid.info.origin.orientation.z = q.z();
            ogrid.info.origin.orientation.w = q.w();
            ogrid.data.resize(m_current_map.width()*m_current_map.height());
            int index=0;
            yarp::dev::MapGrid2D::XYCell cell;
            for (cell.y=m_current_map.height()-1; cell.y>-1; cell.y--)
                for (cell.x=0; cell.x<m_current_map.width(); cell.x++)
                {
                m_current_map.getOccupancyData(cell,tmp);
                ogrid.data[index++]=(int)tmp;
                }
                      
            m_rosPublisher_occupancyGrid.write();
        }
    }

    bool init_map()
    {
        //parse the ini file
        Bottle mapBottle = m_rf.findGroup("MAP");
        if (mapBottle.isNull())
        {
            yError("MAP group not found,closing");
            return false;
        }

        if (mapBottle.check("map_name") == false)
        {
            yError("map_width param not found,closing");
            return false;
        }
        if (m_iMap==0)
        {
            yError("m_iMap is not valid, mapClient was not opened correctly");
            return false;
        }
        std::string map_name = mapBottle.find("map_name").asString();
        if (m_iMap->get_map(map_name,m_current_map))
        {
            yInfo() << "Map " << map_name << " retrieved from map server.";
            //Map loaded. Do not read all other parameters.
           
            std::vector<yarp::os::ConstString> locList;
            m_iMap->getLocationsList(locList);
            m_locations_array.clear();
            for (auto it=locList.begin(); it!=locList.end(); it++)
            {
                yarp::dev::Map2DLocation loc;
                m_iMap->getLocation(*it, loc);
                m_locations_array.push_back(loc);
            }
            return true;
        }

        if (mapBottle.check("map_width") == false)
        {
            yError("map_width param not found,closing");
            return false;
        }
        m_map_width_in_meters = mapBottle.find("map_width").asDouble();

        if (mapBottle.check("map_height") == false)
        {
            yError("map_height param not found,closing");
            return false;
        }
        m_map_height_in_meters = mapBottle.find("map_height").asDouble();

        if (mapBottle.check("map_resolution") == false)
        {
            yError("map_resolution param not found,closing");
            return false;
        }
        m_map_resolution = mapBottle.find("map_resolution").asDouble();
        m_current_map.setMapName(map_name);
        m_current_map.setResolution(m_map_resolution);
        m_current_map.setSize_in_meters(m_map_width_in_meters,m_map_height_in_meters);

        //initialize the map cells as unknown
        for (size_t y=0; y< m_current_map.height(); y++)
            for (size_t x=0; x< m_current_map.width(); x++)
                {
                    m_current_map.setMapFlag(MapGrid2D::XYCell(x,y),MapGrid2D::MAP_CELL_UNKNOWN);
                }

        yInfo() << "created new map with name" << map_name;
        return true;
    }

    void register_scan_to_map()
    {
        //transform the laser measurements in a map
        for (size_t i=0; i< m_laser_map_cells_array.size(); i++)
        {
            m_current_map.setMapFlag(m_laser_map_cells_array[i],MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE);
        }
        m_map_changed=true;
    }
    
    void  readLocalizationData()
    {
        double current_time = yarp::os::Time::now();
        if (m_use_localization_from_odometry_port)
        {
            yarp::sig::Vector *loc = m_port_odometry_input.read(false);
            if (loc)
            {
                m_last_odometry_data_received = yarp::os::Time::now();
                m_localization_data.x = loc->data()[0];
                m_localization_data.y = loc->data()[1];
                m_localization_data.theta = loc->data()[2];
            }
            if (current_time - m_last_odometry_data_received > 0.1)
            {
                yWarning() << "No localization data recevied for more than 0.1s!";
            }
        }
        else if (m_use_localization_from_tf)
        {
            yarp::sig::Vector iv;
            yarp::sig::Vector pose;
            iv.resize(6, 0.0);
            pose.resize(6, 0.0);
            if (m_iTf==0)
            {
                yDebug() << "m_iTf=0";
                return;
            }
            bool r = m_iTf->transformPose(m_frame_robot_id, m_frame_map_id, iv, pose);
            if (r)
            {
                //data is formatted as follows: x, y, angle (in degrees)
                m_tf_data_received = yarp::os::Time::now();
                m_localization_data.x = pose[0];
                m_localization_data.y = pose[1];
                m_localization_data.theta = pose[5] * RAD2DEG;
            }
            if (current_time - m_tf_data_received > 0.1)
            {
                yWarning() << "No localization data received for more than 0.1s!";
            }
        }
        else
        {
            yWarning() << "Localization disabled";
        }
    }

    void  drawMenu(IplImage *map)
    {
        if (map==0) return;
        CvScalar color1 = cvScalar(0, 0, 0);
        CvScalar color2 = cvScalar(255, 0, 0);
        CvScalar color3 = cvScalar(0, 200, 0);
        CvScalar color4 = cvScalar(0, 200, 200);
        cvRectangle(map, m_menu_lu[0], m_menu_rd[0], color1, CV_FILLED);
        cvRectangle(map, m_menu_lu[1], m_menu_rd[1], color2, CV_FILLED);
        cvRectangle(map, m_menu_lu[2], m_menu_rd[2], color3, CV_FILLED);
        cvRectangle(map, m_menu_lu[3], m_menu_rd[3], color4, CV_FILLED);
    }

    void  readLaserData()
    {
        if (m_iLaser==0)
        {
            yDebug() << "iLaser=0";
            return;
        }

        std::vector<LaserMeasurementData> scan;
        bool ret = m_iLaser->getLaserMeasurement(scan);

        if (ret)
        {
            m_laser_map_cells_array.clear();
            unsigned int scansize = scan.size();
            for (unsigned int i = 0; i<scansize; i++)
            {
                double las_x = 0;
                double las_y = 0;
                scan[i].get_cartesian(las_x, las_y);
                //performs a rotation from the robot to the world reference frame
                MapGrid2D::XYWorld world;
                double ss = sin(m_localization_data.theta * DEG2RAD);
                double cs = cos(m_localization_data.theta * DEG2RAD);
                world.x = las_x*cs - las_y*ss + m_localization_data.x;
                world.y = las_x*ss + las_y*cs + m_localization_data.y;
            //    if (!std::isinf(world.x) &&  !std::isinf(world.y))
                if (std::isfinite(world.x) && std::isfinite(world.y))
                   { m_laser_map_cells_array.push_back(m_current_map.world2Cell(world));}
            }
            m_laser_timeout_counter = 0;
        }
        else
        {
            m_laser_timeout_counter++;
        }
    }

    void drawLocations(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
    {
        if (map == 0) return;
        cvCircle(map, cvPoint(current.x, current.y), 3, color);
        if (std::isnan(angle)==false)
        {
            int orient_x = current.x + int(6 * cos(-angle));
            int orient_y = current.y + int(6 * sin(-angle));
            cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
        }
    }

    void draw_map()
    {
         yarp::sig::ImageOf<yarp::sig::PixelRgb> map_image;
         m_current_map.getMapImage(map_image);

#if 0
         IplImage* iplimage = (IplImage*) map_image.getIplImage();
         CvScalar color = cvScalar(0, 200, 0);

         for (size_t y=0; y<m_current_map.height(); y++)
            for (size_t x=0; x<m_current_map.width(); x++)
            {
                MapGrid2D::map_flags flag;
                m_current_map.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag);
                if (flag==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE ||
                    flag==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
                {
                    cvCircle(iplimage, cvPoint(x,y), 0, color);
                }
            }
#endif
        
    }

    void drawCurrentPosition(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
    {
        if (map==0) return;
        cvCircle(map, cvPoint(current.x, current.y), 6, color);
        int orient_x = current.x + int(12 * cos(-angle));
        int orient_y = current.y + int(12 * sin(-angle));
        cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
    }

    void drawLaserScan(IplImage *map, std::vector <MapGrid2D::XYCell>& laser_scan, const CvScalar& color)
    {
        if (map==0) return;
        for (unsigned int i=0; i<laser_scan.size(); i++)
            {cvCircle(map, cvPoint(laser_scan[i].x, laser_scan[i].y), 0, color);}
    }

    void clear_robot_way()
    {
         MapGrid2D::XYCell robot_loc = m_current_map.world2Cell(MapGrid2D::XYWorld(m_localization_data.x, m_localization_data.y));
        m_current_map.setMapFlag(MapGrid2D::XYCell(robot_loc.x  ,robot_loc.y), yarp::dev::MapGrid2D::MAP_CELL_FREE);
        m_current_map.setMapFlag(MapGrid2D::XYCell(robot_loc.x+1,robot_loc.y), yarp::dev::MapGrid2D::MAP_CELL_FREE);
        m_current_map.setMapFlag(MapGrid2D::XYCell(robot_loc.x  ,robot_loc.y+1), yarp::dev::MapGrid2D::MAP_CELL_FREE);
        m_current_map.setMapFlag(MapGrid2D::XYCell(robot_loc.x+1,robot_loc.y+1), yarp::dev::MapGrid2D::MAP_CELL_FREE);
        //m_map_changed=true;
    }

    virtual bool updateModule()
    {
        double current_time = yarp::os::Time::now();

        if (current_time - m_last_statistics_printed > 10.0)
        {
            printStats();
            m_last_statistics_printed = yarp::os::Time::now();
        }

        LockGuard lock(m_mutex);

        readTargetFromYarpView();
        if (m_localization_requested)
        {
            readLocalizationData();
            if (m_clear_robot_way_enabled)
            {
                clear_robot_way();
            }
        }
        if (m_laser_requested)
        {
            readLaserData();
        }

        if (m_mapping_running)
        {
            register_scan_to_map();
        }
        else
        {
            //do nothing
        }
        
        //draw and send on port
        draw_map();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> map_image;
        m_current_map.getMapImage(map_image);
        IplImage* iplimage = (IplImage*) map_image.getIplImage();
            
        if (m_localization_requested)
        {
            MapGrid2D::XYCell start = m_current_map.world2Cell(MapGrid2D::XYWorld(m_localization_data.x, m_localization_data.y));
            CvScalar color_green = cvScalar(0, 200, 0);
            drawCurrentPosition(iplimage, start, m_localization_data.theta*DEG2RAD, color_green);
        }
        
        if (m_laser_requested)
        {
            CvScalar color_blue = cvScalar(0, 0, 200);
            drawLaserScan(iplimage,m_laser_map_cells_array,color_blue);
        }
        
        {
            CvScalar color_blue = cvScalar(100, 100, 200);
            for (auto it=m_locations_array.begin(); it!=m_locations_array.end(); it++)
            {
                drawLocations(iplimage,m_current_map.world2Cell(MapGrid2D::XYWorld(it->x, it->y)),it->theta, color_blue);
            }
        }

        //drawMenu(iplimage);

        //send image on port
        if (m_port_map_output.getOutputCount()>0)
        {
            m_port_map_output.prepare() = map_image;
            m_port_map_output.write();
        }

        //publish map
        double c_time = yarp::os::Time::now();
        static double last_publish_time = yarp::os::Time::now();
        if (m_map_changed && m_map_publish_period>0 && (c_time-last_publish_time)>m_map_publish_period)
        {
            publish_map();
            last_publish_time=c_time;
            m_map_changed=false;
        }
        return true; 
    }

    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).isString() && command.get(0).asString() == "save_to_file")
        {
            std::string filename = command.get(1).asString();
            m_current_map.saveToFile(filename);
            m_mapping_running = false;
            reply.addString("done");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "save_to_mapserver")
        {
            if (command.size()>1)
            {
                std::string mapname = command.get(1).asString();
                if (mapname != "")
                {
                    m_current_map.setMapName(mapname);
                    m_iMap->store_map(m_current_map);
                }
                else
                {
                    yError() << "Missing map name";
                }
            }
            m_mapping_running = false;
            reply.addString("done");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "load_from_file")
        {
            std::string filename = command.get(1).asString();
            m_current_map.loadFromFile(filename);
            m_mapping_running = false;
            reply.addString("done");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "load_from_mapserver")
        {
            std::string mapname = command.get(1).asString();
            m_iMap->get_map(mapname,m_current_map);
            m_mapping_running = false;
            reply.addString("done");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "start")
        {
            m_mapping_running = true;
            m_map_changed = true;
            reply.addString("done");
        }
       
        else if (command.get(0).isString() && command.get(0).asString() == "stop")
        {
            m_mapping_running = false;
            m_map_changed = true;
            std::string filename = command.get(1).asString();
            reply.addString("done");
        }
#if 0
        else if (command.get(0).isString() && command.get(0).asString() == "initLoc")
        {
            yarp::dev::Map2DLocation loc;
            loc.map_id = command.get(1).asString();
            loc.x = command.get(2).asDouble();
            loc.y = command.get(3).asDouble();
            loc.theta = command.get(4).asDouble();
            std::string s = std::string("Localization initialized to: ") + loc.toString();
            reply.addString(s);
        }
#endif 
        else if (command.get(0).isString() && command.get(0).asString() == "set_pixel_type")
        {
            std::string choice = command.get(1).asString();
            if (choice == "wall")
            { 
                m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_WALL;
                reply.addString("wall selected");
            }
            else if (choice == "free")
            { 
                m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_FREE;
                reply.addString("free selected");
            }
            else if (choice == "unknown")
            { 
                m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_UNKNOWN;
                reply.addString("unknown selected");
            }
            else if (choice == "keepout")
            { 
                m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_KEEP_OUT;
                reply.addString("keepout selected");
            }
            else
            {
                m_selected_flag = yarp::dev::MapGrid2D::MAP_CELL_UNKNOWN;
                reply.addString("Invalid string, unknown selected");
            }
        }
        else if (command.get(0).isString() && command.get(0).asString() == "set_pixel_size")
        {
            int pix = command.get(1).asInt();
            if (pix>3 || pix <=0)
            {
                m_pixel_size=1;
            }
            else
            {
                m_pixel_size=pix;
            }
            std::string s = std::string("Pixel size set to: ") + std::to_string(m_pixel_size);
            reply.addString(s);
        }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

     virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        yarp::os::LockGuard lock(m_mutex);
        reply.clear(); 
        if (command.get(0).isString())
        {
            if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("save_to_file <file_name>");
                reply.addString("save_to_mapserver <map_name>");
                reply.addString("start");
                reply.addString("stop");
                reply.addString("load_from_file <file_name>");
                reply.addString("load_from_mapserver <map_name>");
            }
            else if (command.get(0).isString())
            {
                parse_respond_string(command, reply);
            }
        }
        else
        {
            yError() << "Invalid command type";
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("mapper2D.ini");           //overridden by --from parameter
    rf.setDefaultContext("mapper2D");                  //overridden by --context parameter
    rf.configure(argc,argv);
    std::string debug_rf = rf.toString();

    mapperModule mapper;

    return mapper.runModule(rf);
}

 
