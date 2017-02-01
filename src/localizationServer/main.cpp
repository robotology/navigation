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
#include <yarp/dev/Polydriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IFrameTransform.h>

#include <math.h>

using namespace yarp::os;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RAD2DEG 180/M_PI

class localizationModule : public yarp::os::RFModule
{
protected:
    std::string                  m_module_name;
    yarp::dev::Map2DLocation     m_initial_loc;
    yarp::dev::Map2DLocation     m_localization_data;
    yarp::dev::PolyDriver        m_ptf;
    yarp::os::ResourceFinder     m_rf;
    yarp::os::Port               m_rpcPort;
    yarp::os::Mutex              m_mutex;
    bool                         m_use_localization_from_odometry_port;
    bool                         m_use_localization_from_tf;
    std::string                  m_port_broadcast_odometry_name;
    yarp::os::BufferedPort<yarp::sig::Vector>  m_port_odometry_input;
    double                       m_last_odometry_data_received;
    yarp::dev::IFrameTransform*  m_iTf;
    double                       m_tf_data_received;
    std::string                  m_frame_robot_id;
    std::string                  m_frame_map_id;
    double                       m_last_statistics_printed;

public:
    localizationModule()
    {
        m_module_name = "localizationServer";
        m_tf_data_received = -1;
        m_last_odometry_data_received = -1;
        m_last_statistics_printed = -1;

        m_localization_data.map_id = "unknown";
        m_localization_data.x = nan("");
        m_localization_data.y = nan("");
        m_localization_data.theta = nan("");
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

        Bottle initial_group = m_rf.findGroup("INITIAL_POS");
        if (initial_group.isNull())
        {
            yError() << "Missing INITIAL_POS group!";
            return false;
        }

        Bottle localization_group = m_rf.findGroup("LOCALIZATION");
        if (localization_group.isNull())
        {
            yError() << "Missing LOCALIZATION group!";
            return false;
        }

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

        //localization group
        if (localization_group.check("use_localization_from_port")) { m_use_localization_from_odometry_port = (localization_group.find("use_localization_from_port").asInt() == 1); }
        if (localization_group.check("use_localization_from_tf"))   { m_use_localization_from_tf = (localization_group.find("use_localization_from_tf").asInt() == 1); }

        if (m_use_localization_from_odometry_port == true && m_use_localization_from_tf == true)
        {
            yError() << "`use_localization_from_tf` and `use_localization_from_port` cannot be true simulteneously!";
            return false;
        }

        //general group
        if (general_group.check("module_name") == false)
        {
            yError() << "Missing `module_name` in [GENERAL] group";
            return false;
        }
        m_module_name = general_group.find("module_name").asString();

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

        //initial location intialization
        if (initial_group.check("initial_x"))     { m_initial_loc.x = initial_group.find("initial_x").asDouble(); }
        else { yError() << "missing initial_x param"; return false; }
        if (initial_group.check("initial_y"))     { m_initial_loc.y = initial_group.find("initial_y").asDouble(); }
        else { yError() << "missing initial_y param"; return false; }
        if (initial_group.check("initial_theta")) { m_initial_loc.theta = initial_group.find("initial_theta").asDouble(); }
        else { yError() << "missing initial_theta param"; return false; }
        if (initial_group.check("initial_map"))   { m_initial_loc.map_id = initial_group.find("initial_map").asString(); }
        else { yError() << "missing initial_map param"; return false; }
        this->initializeLocalization(m_initial_loc);

        return true;
    }

    virtual bool interruptModule()
    {
        m_rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        m_rpcPort.interrupt();
        m_rpcPort.close();

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

    virtual bool updateModule()
    {
        double current_time = yarp::os::Time::now();

        if (current_time - m_last_statistics_printed > 10.0)
        {
            printStats();
            m_last_statistics_printed = yarp::os::Time::now();
        }

        LockGuard lock(m_mutex);
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
                yWarning() << "No localization data recevied for more than 0.1s!";
            }
        }
        else
        {
            yWarning() << "Localization disabled";
            return false;
        }

        return true; 
    }

    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).isString() && command.get(0).asString() == "getLoc")
        {
            std::string s = std::string("Current Location is: ") + m_localization_data.toString();
            reply.addString(s);
        }

        else if (command.get(0).isString() && command.get(0).asString() == "initLoc")
        {
            yarp::dev::Map2DLocation loc;
            loc.map_id = command.get(1).asString();
            loc.x = command.get(2).asDouble();
            loc.y = command.get(3).asDouble();
            loc.theta = command.get(4).asDouble();
            initializeLocalization(loc);
            std::string s = std::string("Localization initialized to: ") + loc.toString();
            reply.addString(s);
        }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

    bool initializeLocalization(yarp::dev::Map2DLocation& loc)
    {
        m_localization_data.map_id = loc.map_id;
        m_localization_data.x = loc.x;
        m_localization_data.y = loc.y;
        m_localization_data.theta = loc.theta;
        //this value has to be sent to the localization algorithm
        return true;
    }

    bool parse_respond_vocab(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        int request = command.get(1).asVocab();
        if (request == VOCAB_NAV_GET_CURRENT_POS)
        {
            //plannerThread->setNewAbsTarget(loc);
            reply.addVocab(VOCAB_OK);
            reply.addString(m_localization_data.map_id);
            reply.addDouble(m_localization_data.x);
            reply.addDouble(m_localization_data.y);
            reply.addDouble(m_localization_data.theta);
        }
        else if (request == VOCAB_NAV_SET_INITIAL_POS)
        {
            yarp::dev::Map2DLocation loc;
            loc.map_id = command.get(2).asString();
            loc.x = command.get(3).asDouble();
            loc.y = command.get(4).asDouble();
            loc.theta = command.get(5).asDouble();
            initializeLocalization(loc);
            reply.addVocab(VOCAB_OK);
        }
        else
        {
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        yarp::os::LockGuard lock(m_mutex);
        reply.clear(); 
        if (command.get(0).isVocab())
        {
            if(command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
            {
                parse_respond_vocab(command,reply);
            }
            else
            {
                yError() << "Invalid vocab received";
                reply.addVocab(VOCAB_ERR);
            }
        }
        else if (command.get(0).isString())
        {
            if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("getLoc");
                reply.addString("initLoc <map_name> <x> <y> <angle in degrees>");
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
    rf.setDefaultConfigFile("localization.ini");           //overridden by --from parameter
    rf.setDefaultContext("localization");                  //overridden by --context parameter
    rf.configure(argc,argv);
    std::string debug_rf = rf.toString();

    localizationModule robotLocalizer;

    return robotLocalizer.runModule(rf);
}

 
