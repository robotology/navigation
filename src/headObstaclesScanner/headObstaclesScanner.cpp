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


using namespace std;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;



class MyModule : public yarp::os::RFModule
{
    yarp::os::Port handlerPort; // a port to handle messages
    int count;
private:
    PolyDriver *robotDevice;
    IPositionControl *pos;
    IEncoders *encs;

    Vector encoders;
    Vector command;
    Vector tmp;

    double head_speed;
    double rotation_range;
    bool done_run;
    long int com_count = 0;




public:
    std::string  m_remote_localization = "/localizationServer";
    std::string  m_remote_map = "/mapServer";
    std::string  m_remote_navigation = "/navigationServer";
    std::string  m_local_name_prefix = "/headObstaclesScanner";

    PolyDriver      m_pNav;
    yarp::dev::Nav2D::INavigation2D*  m_iNav;



    double getPeriod()
    {
        // module periodicity (seconds), called implicitly by the module.
        return 1;
    }
    // This is our main function. Will be called periodically every getPeriod() seconds
    bool updateModule()
    {
        count++;
        std::cout << "[" << count << "]" << " updateModule..." << '\n';

        while(!encs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }

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
            pos->positionMove(command.data());
        }

        //        for (int ii=0; ii<encoders.length(); ii++)
        //            std::cout << encoders(ii) << " ";
        //        //now set the head to move left and right
        //        std::cout << done_run << std::endl;

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
        if (!handlerPort.open("/myModule"))
            return false;

        // optional, attach a port to the module
        // so that messages received from the port are redirected
        // to the respond method
        attach(handlerPort);

        // CONFIGURATION PARAMETERS

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

        if (!general_group.check("head_speed"))
        {
            head_speed = 30.0;
        }
        else
        {
            head_speed=general_group.find("head_speed").asDouble();
        }

        if (!general_group.check("rotation_range"))
        {
            rotation_range = 25.0;
        }
        else
        {
            rotation_range=general_group.find("rotation_range").asDouble();
        }

        std::string headModeName;
        if (!general_group.check("head_mode"))
        {
            fprintf(stderr, "WARNING parameter head_mode not specified, set default: sweep (other modes: --head_mode sweep, trajectory)\n");
            headModeName = "sweep";
        }
        else
        {
            headModeName=general_group.find("head_mode").asString();
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
        ok = robotDevice->view(pos);
        ok = ok && robotDevice->view(encs);

        if (!ok) {
            printf("Problems acquiring interfaces\n");
            return 0;
        }

        // get axes
        int nj=0;
        pos->getAxes(&nj);
        encoders.resize(nj);
        tmp.resize(nj);
        command.resize(nj);

        // set accelerations and speeds
        int i;
        for (i = 0; i < nj; i++) {
             tmp[i] = 50.0;
        }
        pos->setRefAccelerations(tmp.data());

        for (i = 0; i < nj; i++) {
            tmp[i] = head_speed;
            pos->setRefSpeed(i, tmp[i]);
        }

        //fisrst read all encoders
        //
        printf("waiting for encoders");
        while(!encs->getEncoders(encoders.data()))
        {
            Time::delay(0.1);
            printf(".");
        }
        printf("\n;");

        command=encoders;

        //now set the head to a neutral position
        command[0]=0;
        command[1]=0;
        pos->positionMove(command.data());

        bool done=false;
        while(!done)
        {
            pos->checkMotionDone(&done);
            Time::delay(0.1);
        }


        //open the navigation interface
        if (headModeName="trajectory")
        {
            Bottle navigation_group = rf.findGroup("NAVIGATION");
            if (navigation_group.isNull())
            {
                yWarning() << "Missing NAVIGATION group!";
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
        // optional, close port explicitly
        std::cout << "Calling close function\n";
        robotDevice->close();
        handlerPort.close();
        return true;
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
