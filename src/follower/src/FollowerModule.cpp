/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file FollowerModule.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>

#include "FollowerModule.h"

#ifdef ASSISTIVE_REHAB_AVAILABLE
#include "Person3DPointRetriever.h"
#endif

#include "Ball3DPointRetriever.h"

#include "HumanModel3DPointRetriever.h"

#include <yarp/os/Time.h>


namespace FollowerTarget
{
static const double durationStatInfo_MinVal = -1.0;
static const double durationStatInfo_MaxVal = 1000.0;
static const double durationStatInfo_Count = 100;
}

using namespace std;
using namespace yarp::os;
using namespace FollowerTarget;

//------------------------ buffer helper test ---------------------

double FollowerModule::getPeriod()
{
    // module periodicity (seconds), called implicitly by the module.
    return m_period;
}


// This is our main function. Will be called periodically every getPeriod() seconds
bool FollowerModule::updateModule()
{
    double t=yarp::os::Time::now();
    Target_t targetpoint(ReferenceFrameOfTarget_t::mobile_base_body_link);

    targetpoint = m_pointRetriever_ptr->getTarget();
    m_followerResult= m_follower.followTarget(targetpoint);
    double diff = yarp::os::Time::now()-t;

    if((m_useDurationStatInfo)&&(diff!= 0))
    {
        m_statInfo.addVal(diff);
        if(m_statInfo.countMaxReached())
        {
            yDebug() << "-----DURATION STAT INFO: avg="<<m_statInfo.calculateAvg() <<"min="<<m_statInfo.getMin()<< "max=" <<m_statInfo.getMax() << "-----";
            m_statInfo.reset();
        }
    }

    return true;
}


// Message handler. Just echo all received messages.
bool FollowerModule::respond(const Bottle& command, Bottle& reply)
{
    reply.clear();
    if (command.get(0).asString()=="help")
    {
        if(command.size()>1)
        {
            if(command.get(1).asString() == "debug")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("debug <level> <true/false>");
                reply.addString("   Debug levels are:");
                reply.addString("   general=1");
                reply.addString("   targetRetriever=2");
                reply.addString("   gazeController=3");
                reply.addString("   navigationController=4");
                reply.addString("   ObstacleVerifier=5");
                reply.addString("   DurationStatisticsInfo=6");
                reply.addString("      debug 6 true <count>");

            }
            else
            {
                reply.addString("Unknown command");
            }
        }
        else
        {
        reply.addVocab(Vocab::encode("many"));
        reply.addString("Available commands are:");
        reply.addString("start");
        reply.addString("stop");
        reply.addString("help_provided");
        reply.addString("debug <level> <true/false>");
        }

        return true;
    }
    else if(command.get(0).asString()=="start")
    {
        m_follower.start();
        reply.addString("OK.started");
    }
    else if(command.get(0).asString()=="stop")
    {
        reply.addString("OK.stopped");
        m_follower.stop();
    }
    else if(command.get(0).asString()=="setGazeTimeout")
    {
        reply.addString("OK.setGazeTimeout");
    }
    else if(command.get(0).asString()=="help_provided")
    {
        reply.addString("OK.help_provided");
        m_follower.helpProvided();
    }
    else if(command.get(0).asString() == "debug")
    {
        DebugLevel_t level=static_cast<DebugLevel_t>(command.get(1).asInt());
        bool on=command.get(2).asBool();
        if((level == DebugLevel_t::targetRetriever) && (m_pointRetriever_ptr != nullptr))
            m_pointRetriever_ptr->setDebug(on);
        else if(level == DebugLevel_t::DurationStatisticsInfo)
        {
            int count=command.get(3).asInt32();
            if((on)&&(count>0))
            {
                m_statInfo.init(count, durationStatInfo_MinVal, durationStatInfo_MaxVal);
                m_useDurationStatInfo=true;
            }
            else
                m_useDurationStatInfo=false;
        }
        else
            m_follower.setDebug(level, on);

        reply.addString("OK.debug");
    }
    else
    {
        reply.addString("you");
        reply.addString("said");
        reply.append(command);
    }
    m_rpcPort.reply(reply);
    return true;

}



// Configure function. Receive a previously initialized
// resource finder object. Use it to configure your module.
// If you are migrating from the old module, this is the function
// equivalent to the "open" method.
bool FollowerModule::configure(yarp::os::ResourceFinder &rf)
{

    // 1) configure the follower
    if(!m_follower.configure(rf))
    {
        yError() << "Error reading configuration file";
        return false;
    }

    m_targetType=m_follower.getTargetType();

    // 2) read period and input port of this module from config file
    std::string inputPortName;
    Bottle config_group = rf.findGroup("GENERAL");
    if (config_group.isNull())
    {
        yError() << "Missing GENERAL group! the module uses default value!";
    }
    else
    {
        if (config_group.check("period")){ m_period = config_group.find("period").asFloat64();}
        if (config_group.check("inputPort"))  {inputPortName = config_group.find("inputPort").asString(); }
    }


    bool debugOn = false;
    Bottle debug_group = rf.findGroup("DEBUG");
    if (!debug_group.isNull())
    {
        if (debug_group.check("enable"))  { debugOn = debug_group.find("enable").asBool(); }
        if (debug_group.check("durationInfoStat_count"))
        {
            int count= debug_group.find("durationInfoStat_count").asInt32();
            if(count>0)
            {
                m_useDurationStatInfo=true;
                m_statInfo.init(count, durationStatInfo_MinVal, durationStatInfo_MaxVal);
            }
        }
    }


    // 3) initialize the target retriever
    if(m_targetType == TargetType_t::redball)
    {
        m_pointRetriever_ptr = std::make_unique<Ball3DPointRetriever>();
    }
#ifdef ASSISTIVE_REHAB_AVAILABLE
    else if (m_targetType == TargetType_t::person)
    {
        m_pointRetriever_ptr = std::make_unique<Person3DPointRetriever>();
    }
#endif 
    else if (m_targetType == TargetType_t::fakeHumanModel)
    {
        m_pointRetriever_ptr = std::make_unique<HumanModel3DPointRetriever>();
    }
    else
    {
        yError() << "m_targetType not available!";
        return false;
    }


    if(! m_pointRetriever_ptr->init(rf))
    {
        yError() << "Error in initializing the Target Retriever";
        return false;
    }

    m_rpcPort.open("/follower/rpc");
    attach(m_rpcPort);
    #ifdef TICK_SERVER
    TickServer::configure_tick_server("/follower");
    #endif

    return true;
}
// Interrupt function.
bool FollowerModule::interruptModule()
{
    m_rpcPort.interrupt();
    m_rpcPort.close();

    if(m_pointRetriever_ptr!=nullptr)
        m_pointRetriever_ptr->deinit();

    m_follower.close();
    return true;
}
// Close function, to perform cleanup.
bool FollowerModule::close()
{
    if(m_pointRetriever_ptr!=nullptr)
        m_pointRetriever_ptr->deinit();

    m_pointRetriever_ptr.reset();

    m_follower.close();

    return true;
}


FollowerModule::FollowerModule():m_period(DefaultPeriodOfMudule), m_targetType(TargetType_t::person), m_statInfo(durationStatInfo_Count, durationStatInfo_MinVal, durationStatInfo_MaxVal), m_followerResult(Result_t::notRunning), m_useDurationStatInfo(false)
{}
FollowerModule::~FollowerModule(){;}


#ifdef TICK_SERVER
ReturnStatus FollowerModule::request_tick(const std::string& params)
{
    m_follower.start();
    return ReturnStatus::BT_RUNNING;
}

ReturnStatus FollowerModule::request_status()
{
    switch(m_followerResult)
    {
        case(Result_t::ok):
        case(Result_t::lostTarget):
        case(Result_t::autoNavigation):
            {return ReturnStatus::BT_RUNNING;}

        case(Result_t::notRunning): {return ReturnStatus::BT_HALTED;}
        case(Result_t::failed): {return ReturnStatus::BT_FAILURE;}
        case(Result_t::error): {return ReturnStatus::BT_ERROR;}
        default:{return ReturnStatus::BT_ERROR;}
    };
}

ReturnStatus FollowerModule::request_halt()
{
    m_follower.stop();
    return ReturnStatus::BT_HALTED;
}

#endif


//------------------------------------------------
// private function
//------------------------------------------------
