/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include <string>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/ILocalization2D.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "cornerdetector.h"

#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/cv/Cv.h>



#ifndef M_PI
#define M_PI 3.14159265
#endif

const double RAD2DEG = 180.0 / M_PI;
const double DEG2RAD = M_PI / 180.0;

#define TIMEOUT_MAX 100

// FOR DEBUGGING INFO
#define DEBUG

using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

class MyModule : public yarp::os::RFModule
{
    // FAST SETUP:
        // ./headObstaclesScanner --GENERAL::robot R1 --GENERAL::head_mode closer_corner
        // to see image debug:
        // yarpview --name /headObstaclesScanner/debug:i
        // yarp connect /headObstaclesScanner/rgb:o /headObstaclesScanner/debug:i

    // PARAMETERS

    // head behaviour default parameters
    double head_speed = 30.0;
    double rotation_range = 25.0;
    double circle_range = 1;

    // devices default parameters
    std::string  m_remote_localization = "/localizationServer";
    std::string  m_remote_map = "/mapServer";
    std::string  m_remote_navigation = "/navigationServer";
    std::string  m_local_name_prefix = "/headObstaclesScanner";
    std::string  headModeName = "sweep";
    //std::string  map_name = "/dockersharedfolder/testopencv/test4/squirico_map.png";
    std::string  map_name = "/dockersharedfolder/testopencv/test4/mymap_isaac3.png";
    double  map_resolution = 0.02;



    // DEVICES
    PolyDriver *robotDevice;
    IPositionControl *ipos;
    IEncoders *iencs;
    IPositionDirect *idirect;
    IControlMode *icontrolMode;

    PolyDriver      m_pNav;
    PolyDriver      m_pLoc;

    yarp::dev::Nav2D::INavigation2D*        m_iNav;
    yarp::dev::Nav2D::ILocalization2D*      m_iLoc;

    yarp::dev::Nav2D::Map2DPath  m_all_waypoints;
    yarp::dev::Nav2D::Map2DLocation        m_localization_data;
    yarp::dev::Nav2D::Map2DLocation        m_target_data;

    // SERVICE VARIABLES
    cornerDetector *cornersMapObj;

    yarp::os::Port handlerPort; // a port to handle messages
    int count;

    Vector encoders;
    Vector command;
    Vector tmp;

    bool done_run;
    long int com_count = 0;
    int m_loc_timeout_counter = 0;

    yarp::sig::Matrix abs_waypoints;
    yarp::sig::Matrix rel_waypoints;
    yarp::sig::Matrix map_corners;
    vector<Point2f> opencv_corners;
    yarp::sig::Matrix rel_map_corners;
    yarp::sig::Matrix robot_pose;
    std::vector<double> waypoint_distance;
    Vector relative_target_loc = {0, 0, 0};
    yarp::dev::Nav2D::NavigationStatusEnum  nav_status;
    std::vector<double> looking_point;

    char* source_window = "Image";
    Mat copy;
    Mat src, src_gray;

    BufferedPort<ImageOf<PixelRgb> > imagePort;

    int closer_point_index = 0;

    yarp::conf::vocab32_t VOCAB_CM_POSITION        =   yarp::os::createVocab('p','o','s');
    yarp::conf::vocab32_t VOCAB_CM_POSITION_DIRECT =   yarp::os::createVocab('p','o','s','d');



    //int *actualMode;

    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 0.5;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        count++;
        std::cout << "[" << count << "]" << " updateModule..." << '\n';


        //head encoders reading
        while(!iencs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }

        // head motion mode: sweep
        if (headModeName=="sweep")
        {
            sweepMode();
        }

        // head motion mode: trajectory
        if (headModeName=="trajectory")
        {
            double abs_angle=0;
            double rel_angle=0;
            bool short_trajectory = true;

            // get robot position
            if (!getRobotPosition())
                return false;

            // get trajectory
            getTrajectory();

            // calculate relative position
            for (int i=0; i<abs_waypoints.rows(); i++)
            {
                for (int j=0; j<robot_pose.cols(); j++)
                    rel_waypoints(i,j) = abs_waypoints(i,j) - robot_pose(0,j);
            }

            for (int j=0; j<robot_pose.cols(); j++)
                relative_target_loc[j] = relative_target_loc[j] - robot_pose(0,j);

            // calculate intersection with desired path
            if (rel_waypoints.rows() > 1)
            {
                waypoint_distance.resize(rel_waypoints.rows());
                for (int i=0; i<rel_waypoints.rows(); i++)
                {
                    waypoint_distance[i] = sqrt(pow(rel_waypoints(i,0),2) + pow(rel_waypoints(i,1),2));
                    if (waypoint_distance[i] > circle_range)
                    {
                        //abs_angle = atan2(rel_waypoints(i,1), rel_waypoints(i,0));
                        looking_point[0] = rel_waypoints(i,0);
                        looking_point[1] = rel_waypoints(i,1);
                        short_trajectory = false;
                        break;
                    }
                }
                if (short_trajectory)
                {
                   looking_point[0] = relative_target_loc(0);
                   looking_point[1] = relative_target_loc(1);
                   //abs_angle = atan2(relative_target_loc(1), relative_target_loc(0));
                }


//                abs_angle = abs_angle * RAD2DEG;
//                if (abs_angle < 0)
//                    abs_angle = abs_angle + 360;

//                rel_angle = abs_angle - robot_pose(0,2);
            }

            lookPoint();


//            // stop head when target is reached

//            m_iNav->getNavigationStatus(nav_status);
//            if ((nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached) || nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
//                rel_angle = 0;

//            // move robot head
//            command[0]=0;
//            command[1]=rel_angle;

//            //pos control mode: 7565168 ---- pos direct control mode: 1685286768

//            if (std::abs(encoders(1)-command(1)) < 3)
//            {
//                icontrolMode->setControlMode(1,VOCAB_CM_POSITION_DIRECT);
//                idirect->setPosition(1,command[1]);
//            }
//            else
//            {
//                icontrolMode->setControlMode(1,VOCAB_CM_POSITION);
//                ipos->positionMove(command.data());
//            }


#ifdef DEBUG

            std::cout << "relative position" << '\n';
            std::cout << rel_waypoints.toString() << '\n';
            std::cout << "absolute angle: " << abs_angle << '\n';
            std::cout << "relative angle: " << rel_angle << " from final target? " << short_trajectory << '\n';
            std::cout << "relative target: " << relative_target_loc.toString() << '\n';

            //std::cout << "INFO - pos control mode value: " << VOCAB_CM_POSITION << '\n';
            //std::cout << "INFO - pos direct control mode value: " << VOCAB_CM_POSITION_DIRECT << '\n';
            int actualMode=0;
            icontrolMode->getControlMode(1, &actualMode);
            std::cout << "getControlMode: " << actualMode << '\n';
            if (nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached)
                std::cout << "INFO - goal reached \n";
            if (nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
                std::cout << "INFO - idle \n";




            //std::cout << yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached << '\n';
            //std::cout << nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached  << '\n'
            //std::cout << nav_status << '\n';

#endif
        }

        if (headModeName=="closer_corner")
        {
            // get robot position
            if (!getRobotPosition())
                return false;

            // get relative position of the corners
            if (!getRelMapCorners())
                return false;

            // find closer corner
            double temp_distance = 1000000;
            double i_distance;
            for (int i=0; i<rel_map_corners.rows(); i++)
            {
                i_distance = sqrt(pow(rel_map_corners(i,0),2) + pow(rel_map_corners(i,1),2));
                if (i_distance < temp_distance)
                {
                    temp_distance = i_distance;
                    closer_point_index = i;
                }
            }
            cout << "closer_point_index: " << closer_point_index << '\n';

            // show image
            drawImage();

            // look at the closer corner

            looking_point[0] = rel_map_corners(closer_point_index,0);
            looking_point[1] = rel_map_corners(closer_point_index,1);
            lookPoint();

        }

        return true;
    }
    // Message handler. Just echo all received messages.
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        std::cout << "Got something, echo is on" << '\n';
        if (command.get(0).asString() == "quit")
            return false;
        else
            reply = command;
        return true;
    }
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {
        count=0;
        done_run = true;
        robot_pose.resize(1,3);
        robot_pose.zero();

        looking_point.resize(2);
        looking_point[0] = 0;
        looking_point[1] = 0;

        if (!handlerPort.open("/myModule"))
            return false;

        // optional, attach a port to the module
        // so that messages received from the port are redirected
        // to the respond method
        attach(handlerPort);

        // CONFIGURATION PARAMETERS (general)

        Bottle general_group = rf.findGroup("GENERAL");
        if (general_group.isNull())
        {
            yError() << "Missing GENERAL group!";
            return false;
        }

        if (!general_group.check("robot"))
        {
            fprintf(stderr, "Please specify the name of the robot\n");
            fprintf(stderr, "--robot name (e.g. icub)\n");
            return 1;
        }

        if (general_group.check("head_speed"))
        {
            head_speed=general_group.find("head_speed").asDouble();;
        }

        if (general_group.check("rotation_range"))
        {
            rotation_range=general_group.find("rotation_range").asDouble();
        }

        if (!general_group.check("head_mode"))
        {
            fprintf(stderr, "WARNING parameter head_mode not specified, set default (modes: --head_mode sweep, trajectory, closer_corner)\n");
        }
        else
        {
            headModeName=general_group.find("head_mode").asString();
        }

        // CONFIGURATION PARAMETERS (head behaviour)
        Bottle head_group = rf.findGroup("HEAD");
        if (head_group.isNull())
        {
            yWarning() << "Missing HEAD group, default parameters used";
           // return false;
        }

        if (head_group.check("circle_range"))
        {
            circle_range = head_group.find("circle_range").asDouble();
        }

        std::string robotName=general_group.find("robot").asString();
        std::string remotePorts="/";
        remotePorts+=robotName;
        remotePorts+="/head";
        std::string localPorts="/test/client";

        Property options;
        options.put("device", "remote_controlboard");
        options.put("local", localPorts);   //local port names
        options.put("remote", remotePorts); //where we connect to


        // create a device
        //PolyDriver robotDevice(options);
        robotDevice = new PolyDriver(options);

        if (!robotDevice->isValid()) {
            printf("Device not available.  Here are the known devices:\n");
            printf("%s", Drivers::factory().toString().c_str());
            return 0;
        }


        bool ok;
        ok = robotDevice->view(ipos);
        ok = ok && robotDevice->view(iencs);
        ok = ok && robotDevice->view(idirect);
        ok = ok && robotDevice->view(icontrolMode);

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }

        // get axes
        int nj=0;
        ipos->getAxes(&nj);
        encoders.resize(nj);
        tmp.resize(nj);
        command.resize(nj);

        // set accelerations and speeds
        int i;
        for (i = 0; i < nj; i++) {
             tmp[i] = 50.0;
        }
        ipos->setRefAccelerations(tmp.data());

        for (i = 0; i < nj; i++) {
            tmp[i] = head_speed;
            ipos->setRefSpeed(i, tmp[i]);
        }

        //fisrst read all encoders
        //
        printf("waiting for encoders");
        while(!iencs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
        printf("\n;");

        command=encoders;

        //now set the head to a neutral position
        command[0]=0;
        command[1]=0;
        ipos->positionMove(command.data());

        bool done=false;
        while(!done)
        {
            ipos->checkMotionDone(&done);
            Time::delay(0.1);
        }




        if ((headModeName=="trajectory") || (headModeName=="closer_corner"))
        {
            // parameters for localization and navigation servers

            Bottle navigation_group = rf.findGroup("NAVIGATION");
            if (navigation_group.isNull())
            {
                yWarning() << "Missing NAVIGATION group, default parameters used";
               // return false;
            }

            if (general_group.check("local"))
            {
                m_local_name_prefix = general_group.find("local").asString();
            }
            if (navigation_group.check("remote_localization"))
            {
                m_remote_localization = navigation_group.find("remote_localization").asString();
            }
            if (navigation_group.check("remote_navigation"))
            {
                m_remote_navigation = navigation_group.find("remote_navigation").asString();
            }
            if (navigation_group.check("remote_map"))
            {
                m_remote_map = navigation_group.find("remote_map").asString();
            }

            //open the localization interface
            Property loc_options;
            loc_options.put("device", "localization2DClient");
            loc_options.put("local", m_local_name_prefix+"/localizationClient");
            loc_options.put("remote", m_remote_localization);
            if (m_pLoc.open(loc_options) == false)
            {
                yError() << "Unable to open localization driver";
                return false;
            }
            m_pLoc.view(m_iLoc);
            if (m_pLoc.isValid() == false || m_iLoc == 0)
            {
                yError() << "Unable to view localization interface";
                return false;
            }

            //open the navigation interface

            Property nav_options;
            nav_options.put("device", "navigation2DClient");
            nav_options.put("local", m_local_name_prefix + "/navigation2DClient");
            nav_options.put("navigation_server", m_remote_navigation);
            nav_options.put("map_locations_server", m_remote_map);
            nav_options.put("localization_server", m_remote_localization);
            if (m_pNav.open(nav_options) == false)
            {
                yError() << "Unable to open navigation2DClient";
                return false;
            }
            m_pNav.view(m_iNav);
            if (m_iNav == 0)
            {
                yError() << "Unable to open navigation interface";
                return false;
            }
        }

        if (headModeName=="closer_corner")
        {
            // read map
            if (head_group.check("map_name"))
            {
                map_name = head_group.find("map_name").asString();
            }
            if (head_group.check("map_resolution"))
            {
                map_resolution = head_group.find("map_resolution").asDouble();
            }

            // create corner detector obj
            cornersMapObj = new cornerDetector(map_name);

            // get corners
            getRelMapCorners();

            // open image port and send image
            imagePort.open(m_local_name_prefix+"/rgb:o");

            drawImage();

            std::cout << "corners found:  " << rel_map_corners.rows() << '\n';
            std::cout << "relative corners: \n " << rel_map_corners.toString() << '\n';
        }


        return true;
    }
    // Interrupt function.
    bool interruptModule()
    {
        std::cout << "Interrupting your module, for port cleanup" << '\n';
        return true;
    }
    // Close function, to perform cleanup.
    bool close()
    {
        icontrolMode->setControlMode(1,VOCAB_CM_POSITION);
        // optional, close port explicitly
        std::cout << "Calling close function\n";
        robotDevice->close();
        handlerPort.close();


        if (m_pNav.isValid()) m_pNav.close();
        m_iNav = nullptr;


        return true;
    }

    // head mode sweep
    bool sweepMode()
    {
        if (std::abs(encoders(1)-command(1)) < 1)
        {
            done_run = true;
        }
        else
        {
            done_run = false;
        }


        if (done_run)
        {
            com_count ++;
            if (com_count%2)
            {
                command[0]=0;
                command[1]=rotation_range;
            }
            else
            {
                command[0]=0;
                command[1]=-rotation_range;
            }
            ipos->positionMove(command.data());
        }
        return true;
    }

    bool getTrajectory()
    {

        double rel_x =0;
        double rel_y =0;
        double rel_t =0;
        //m_iNav->getRelativeLocationOfCurrentTarget(rel_x, rel_y, rel_t);
        //relative_target_loc[0] = rel_x;
        //relative_target_loc[1] = rel_y;
        //relative_target_loc[2] = rel_t;

        if (m_iNav->getAbsoluteLocationOfCurrentTarget(m_target_data))
        {
            relative_target_loc[0] = m_target_data.x;
            relative_target_loc[1] = m_target_data.y;
            relative_target_loc[2] = m_target_data.theta;
            if (relative_target_loc[2]<0)
                relative_target_loc[2] = relative_target_loc[2] + 360;
        }

        m_iNav->getAllNavigationWaypoints(m_all_waypoints);

        abs_waypoints.resize(m_all_waypoints.size(),3);
        rel_waypoints.resize(m_all_waypoints.size(),3);

        for (int i = 0; i < m_all_waypoints.size(); i++)
        {
            abs_waypoints(i,0) = m_all_waypoints[i].x;
            abs_waypoints(i,1) = m_all_waypoints[i].y;
            abs_waypoints(i,2) = m_all_waypoints[i].theta;
            if (abs_waypoints(i,2)<0)
                abs_waypoints(i,2) = abs_waypoints(i,2) + 360;
        }

#ifdef DEBUG
        std::cout << "trajectory data" << '\n';
        //std::cout << m_all_waypoints.toString() << '\n';
        std::cout << abs_waypoints.toString() << '\n';
#endif

        return true;
    }
      bool getRobotPosition()
    {
        bool ret = m_iLoc->getCurrentPosition(m_localization_data);
        if (ret)
        {
            m_loc_timeout_counter = 0;
            robot_pose(0,0) = m_localization_data.x;
            robot_pose(0,1) = m_localization_data.y;
            robot_pose(0,2) = m_localization_data.theta;
            if (robot_pose(0,2)<0)
                robot_pose(0,2) = robot_pose(0,2) + 360;
        }
        else
        {
            m_loc_timeout_counter++;
            if (m_loc_timeout_counter>TIMEOUT_MAX) m_loc_timeout_counter = TIMEOUT_MAX;
            yError(" timeout, no localization data received!\n");
            return false;
        }

#ifdef DEBUG
        std::cout << "localization data" << '\n';
        //std::cout << m_localization_data.toString() << '\n';
        std::cout << robot_pose.toString() << '\n';
#endif

        return true;
    }

      bool getRelMapCorners()
    {
          cornersMapObj->calculateCorners();
          map_corners = cornersMapObj->yarp_corners;
          rel_map_corners.resize(map_corners.rows() , map_corners.cols());

          for (int i=0; i<map_corners.rows(); i++)
          {
                  rel_map_corners(i,0) = map_corners(i,0)*map_resolution - robot_pose(0,0);
                  rel_map_corners(i,1) = map_corners(i,1)*map_resolution - robot_pose(0,1);
          }
        return true;
    }

      bool drawImage()
    {
          // Read image
          src = cv::imread( map_name, 1 );

          copy = src.clone();
          cv::RNG rng(12345);

          // Create Window
          //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
          //imshow( source_window, src );

          // Draw corners detected
          opencv_corners = cornersMapObj->corners;

          int r = 4;
          for( int i = 0; i < opencv_corners.size(); i++ )
             { circle( copy, opencv_corners[i], r, Scalar(255, 0, 0), -1, 8, 0 ); }

          circle( copy, opencv_corners[closer_point_index], r, Scalar(0, 255, 0), -1, 8, 0 );  // closer corner has a different color

          // Draw robot position
          Point2f opencv_robot_pos;
          opencv_robot_pos.x = robot_pose(0,1)/map_resolution;
          opencv_robot_pos.y = robot_pose(0,0)/map_resolution;

          circle( copy, opencv_robot_pos, r*2, Scalar(0, 0, 255), -1, 8, 0 );


          // Send image to the door
          ImageOf<PixelRgb> &img = imagePort.prepare();

          img = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(copy);

          imagePort.write();

    }

      bool lookPoint()
      {
          double abs_angle = 0;
          double rel_angle;

          abs_angle = atan2(looking_point[1] , looking_point[0]);
          abs_angle = abs_angle * RAD2DEG;
          if (abs_angle < 0)
              abs_angle = abs_angle + 360;

          rel_angle = abs_angle - robot_pose(0,2);

          // stop head when target is reached

          m_iNav->getNavigationStatus(nav_status);
          if ((nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_goal_reached) || nav_status == yarp::dev::Nav2D::NavigationStatusEnum::navigation_status_idle)
              //rel_angle = 0;

          // move robot head
          command[0]=0;

          if (rel_angle > 180 )
              command[1]= rel_angle - 360;
          else
              command[1]= rel_angle;

          //pos control mode: 7565168 ---- pos direct control mode: 1685286768

          if (std::abs(encoders(1)-command(1)) < 3)
          {
              icontrolMode->setControlMode(1,VOCAB_CM_POSITION_DIRECT);
              idirect->setPosition(1,command[1]);
          }
          else
          {
              icontrolMode->setControlMode(1,VOCAB_CM_POSITION);
              ipos->positionMove(command.data());
          }

#ifdef DEBUG
            std::cout << "LOOKING POINT:" << '\n';
            std::cout << "robot position X: " << robot_pose(0,0) << " Y: " << robot_pose(0,1) << " theta: " << robot_pose(0,2) <<'\n';
            std::cout << "relative looking point X: " << looking_point[0] << " Y: " << looking_point[1] << '\n';
            std::cout << "point angle in robot reference system: " << abs_angle << '\n';
            std::cout << "point angle relative to robot head: " << rel_angle << '\n';
            std::cout << "commanded angle: " << command[1] << '\n';
#endif

      }

};

int main(int argc, char * argv[])
{
    // initialize yarp network
    yarp::os::Network yarp;

    // create your module
    MyModule module;
    // prepare and configure the resource finder
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    std::cout << "Configuring and starting module.\n";
    // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    if (!module.runModule(rf)) {
        std::cerr << "Error module did not start\n";
    }

    std::cout << "Main returning..." << '\n';
    return 0;
}
