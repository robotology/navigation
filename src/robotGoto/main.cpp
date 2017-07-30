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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include "robotGoto.h"
#include <math.h>

class robotGotoModule : public yarp::os::RFModule
{
protected:
    GotoThread     *gotoThread;
    yarp::os::Port rpcPort;

public:
    robotGotoModule()
    {
        gotoThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        Property p;
        ConstString configFile = rf.findFile("from");
        if (configFile!="") p.fromConfigFile(configFile.c_str());

        gotoThread = new GotoThread(10,rf,p);

        if (!gotoThread->start())
        {
            delete gotoThread;
            return false;
        }

        rpcPort.open("/robotGoto/rpc");
        attach(rpcPort);
        //attachTerminal();

        return true;
    }

    virtual bool interruptModule()
    {
       // rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        //rpcPort.interrupt();
        rpcPort.close();

        //gotoThread->shutdown();
        gotoThread->stop();
        delete gotoThread;
        gotoThread=NULL;

        return true;
    }

    virtual double getPeriod()
    { 
        if (gotoThread) gotoThread->printStats();
        return 1.0; 
    }
    
    virtual bool updateModule()
    { 
        if (isStopping())
        {
            gotoThread->stop();
            return false;
        }
        
        bool err = false;
        if (gotoThread->m_las_timeout_counter>TIMEOUT_MAX)
        {
            yError (" timeout, no laser data received!");
            err= true;
        }
        if (gotoThread->m_loc_timeout_counter>TIMEOUT_MAX)
        {
            yError(" timeout, no localization data receive");
            err= true;
        }
        
        if (err==false)
        yInfo ("module running, ALL ok");

        return true; 
    }

    bool parse_respond_string(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {
        if (command.get(0).isString() && command.get(0).asString() == "reset_params")
        {
            gotoThread->resetParamsToDefaultValue();
            reply.addString("params reset done");
        }

        else if (command.get(0).isString() && command.get(0).asString() == "approach")
        {
            double dir    = command.get(1).asDouble();
            double speed  = command.get(2).asDouble();
            double time   = command.get(3).asDouble();
            gotoThread->approachTarget(dir,speed,time);
            reply.addString("approach command received");
        }

        else if (command.get(0).isString() && command.get(0).asString() == "gotoAbs")
        {
            yarp::sig::Vector v;
            v.push_back(command.get(1).asDouble());
            v.push_back(command.get(2).asDouble());
            if (command.size() == 4) v.push_back(command.get(3).asDouble());
            gotoThread->setNewAbsTarget(v);
            reply.addString("new absolute target received");
        }

        else if (command.get(0).isString() && command.get(0).asString() == "gotoRel")
        {
            yarp::sig::Vector v;
            v.push_back(command.get(1).asDouble());
            v.push_back(command.get(2).asDouble());
            if (command.size() == 4) v.push_back(command.get(3).asDouble());
            gotoThread->setNewRelTarget(v);
            reply.addString("new relative target received");
        }
        else if (command.get(0).asString() == "set")
        {
            if (command.get(1).asString() == "linear_tol")
            {
                gotoThread->m_goal_tolerance_lin = command.get(2).asDouble();
                reply.addString("linear_tol set.");
            }
            else if (command.get(1).asString() == "angular_tol")
            {
                gotoThread->m_goal_tolerance_ang = command.get(2).asDouble();
                reply.addString("angular_tol set.");
            }
            else if (command.get(1).asString() == "max_lin_speed")
            {
                gotoThread->m_max_lin_speed = command.get(2).asDouble();
                reply.addString("max_lin_speed set.");
            }
            else if (command.get(1).asString() == "max_ang_speed")
            {
                gotoThread->m_max_ang_speed = command.get(2).asDouble();
                reply.addString("max_ang_speed set.");
            }
            else if (command.get(1).asString() == "min_lin_speed")
            {
                gotoThread->m_min_lin_speed = command.get(2).asDouble();
                reply.addString("min_lin_speed set.");
            }
            else if (command.get(1).asString() == "min_ang_speed")
            {
                gotoThread->m_min_ang_speed = command.get(2).asDouble();
                reply.addString("min_ang_speed set.");
            }
            else if (command.get(1).asString() == "obstacle_avoidance")
            {
                if (gotoThread->m_enable_obstacles_avoidance)
                {
                    reply.addString("enable_obstacles_avoidance=false");
                    gotoThread->m_enable_obstacles_avoidance = false;
                }
                else
                {
                    gotoThread->m_enable_obstacles_avoidance = true;
                    reply.addString("enable_obstacles_avoidance=true");
                }
            }
            else if (command.get(1).asString() == "obstacle_stop")
            {
                if (gotoThread->m_enable_obstacles_emergency_stop)
                {
                    reply.addString("enable_obstacle_stop=false");
                    gotoThread->m_enable_obstacles_emergency_stop = false;
                }
                else
                {
                    gotoThread->m_enable_obstacles_emergency_stop = true;
                    reply.addString("enable_obstacle_stop=true");
                }
            }
            else
            {
                reply.addString("Unknown set.");
            }
        }
        else if (command.get(0).asString() == "get")
        {
            if (command.get(1).asString() == "navigation_status")
            {
                string s = gotoThread->getNavigationStatusAsString();
                reply.addString(s.c_str());
            }
            else
            {
                reply.addString("Unknown get.");
            }
        }
        else if (command.get(0).isString() && command.get(0).asString() == "stop")
        {
            gotoThread->stopMovement();
            reply.addString("Stopping movement.");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "pause")
        {
            double time = -1;
            if (command.size() > 1)
                time = command.get(1).asDouble();
            gotoThread->pauseMovement(time);
            reply.addString("Pausing.");
        }
        else if (command.get(0).isString() && command.get(0).asString() == "resume")
        {
            gotoThread->resumeMovement();
            reply.addString("Resuming.");
        }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

    bool parse_respond_vocab(const yarp::os::Bottle& command, yarp::os::Bottle& reply)
    {

        int request = command.get(1).asVocab();

        if (request == VOCAB_NAV_GOTOABS)
        {
            yarp::sig::Vector v;
            v.push_back(command.get(3).asDouble());
            v.push_back(command.get(4).asDouble());
            if (command.size() == 6) v.push_back(command.get(5).asDouble());
            gotoThread->setNewAbsTarget(v);
            reply.addVocab(VOCAB_OK);
        }
        else if (request == VOCAB_NAV_GOTOREL)
        {
            yarp::sig::Vector v;
            v.push_back(command.get(2).asDouble());
            v.push_back(command.get(3).asDouble());
            if (command.size() == 5) v.push_back(command.get(4).asDouble());
            gotoThread->setNewRelTarget(v);
            reply.addVocab(VOCAB_OK);
        }
        else if (request == VOCAB_NAV_GET_STATUS)
        {
            int nav_status = gotoThread->getNavigationStatusAsInt();
            reply.addVocab(VOCAB_OK);
            reply.addInt(nav_status);
        }
        else if (request == VOCAB_NAV_STOP)
        {
            gotoThread->stopMovement();
            reply.addVocab(VOCAB_OK);
        }
        else if (request == VOCAB_NAV_SUSPEND)
        {
            double time = -1;
            if (command.size() > 2)
                time = command.get(2).asDouble();
            gotoThread->pauseMovement(time);
            reply.addVocab(VOCAB_OK);
        }
        else if (request == VOCAB_NAV_RESUME)
        {
            gotoThread->resumeMovement();
            reply.addVocab(VOCAB_OK);
        }
        else if (request == VOCAB_NAV_GET_CURRENT_POS)
        {
            Map2DLocation position;
            gotoThread->getCurrentPos(position);
            reply.addVocab(VOCAB_OK);
            reply.addString(position.map_id);
            reply.addDouble(position.x);
            reply.addDouble(position.y);
            reply.addDouble(position.theta);
        }
        else if (request == VOCAB_NAV_GET_ABS_TARGET || request == VOCAB_NAV_GET_REL_TARGET)
        {
            Map2DLocation loc = request == VOCAB_NAV_GET_ABS_TARGET ? gotoThread->getCurrentAbsTarget() : gotoThread->getCurrentRelTarget();
            reply.addVocab(VOCAB_OK);

            if(request == VOCAB_NAV_GET_ABS_TARGET) reply.addString(loc.map_id);

            reply.addDouble(loc.x);
            reply.addDouble(loc.y);
            reply.addDouble(loc.theta);
        }
        else
        {
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        reply.clear(); 

        gotoThread->m_mutex.wait();
        if (command.get(0).asString()=="quit")
        {
            gotoThread->m_mutex.post();
            return false;
        }

        else if (command.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands are:");
            reply.addString("gotoAbs <x> <y> <angle in degrees>");
            reply.addString("gotoRel <x> <y> <angle in degrees>");
            reply.addString("approach <angle in degrees> <linear velocity> <time>");
            reply.addString("stop");
            reply.addString("pause");
            reply.addString("resume");
            reply.addString("quit");
            reply.addString("reset_params");
            reply.addString("set linear_tol <m>");
            reply.addString("set linear_ang <deg>");
            reply.addString("set max_lin_speed <m/s>");
            reply.addString("set max_ang_speed <deg/s>");
            reply.addString("set min_lin_speed <m/s>");
            reply.addString("set min_ang_speed <deg/s>");
            reply.addString("set obstacle_stop");
            reply.addString("set obstacle_avoidance");
        }
        else if (command.get(0).isString())
        {
            parse_respond_string(command,reply);
        }
        else if (command.get(0).isVocab())
        {
            if(command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
            {
                parse_respond_vocab(command,reply);
            }
            else
            {
                reply.addVocab(VOCAB_ERR);
            }
        }
        else
        {
            yError() << "Invalid command type";
            reply.addVocab(VOCAB_ERR);
        }

        gotoThread->m_mutex.post();
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("ERROR: check Yarp network.");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("robotGoto.ini");           //overridden by --from parameter
    rf.setDefaultContext("robotGoto");                  //overridden by --context parameter
    rf.configure(argc,argv);
    
    robotGotoModule robotGoto;

    return robotGoto.runModule(rf);
}

 
