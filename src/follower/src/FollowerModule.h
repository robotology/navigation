/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file FollowerModule.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <string>
#include <memory>

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/PolyDriver.h>
//#include <yarp/os/RpcClient.h>


#include "TargetRetriever.h"
#include "Follower.h"
#include "BTMonitor.h"
#ifdef TICK_SERVER
#include <tick_server.h>
#include <ReturnStatus.h>
#include <BTMonitorMsg.h>   // include message used for monitoring
#endif

namespace FollowerTarget
{

double const DefaultPeriodOfMudule=0.05;

class StatInfo
{
private:
    double avg;
    double min;
    double max;
    double sum;
    uint32_t count;
    uint32_t count_max;
    double cfg_min;
    double cfg_max;
public:
    StatInfo(uint32_t countMax, double minDef, double maxDef): avg(0), min(maxDef), max(minDef), sum(0), count(0), count_max(countMax), cfg_min(maxDef), cfg_max(minDef){;}

    void init(uint32_t countMax, double minDef, double maxDef)
    {
        avg=sum=count=0;
        min=cfg_min=maxDef;
        max=cfg_max=minDef;
        count_max=countMax;
    }

    double getMin(){return min;}
    double getMax(){return max;}
    bool countMaxReached()
    {
        if(count>=count_max)return true;
        else return false;
    }

    double calculateAvg(){avg=sum/count; sum=0; count=0; return avg;}
    void addVal(double v)
    {
        sum+=v; count++;
        if(v<min) min=v;

        if(v>max) max=v;
    }
    void reset(void) {avg=0; sum=0; count=0; min=cfg_min; max=cfg_max;}
};

#ifdef TICK_SERVER
class FollowerModule:public yarp::os::RFModule, public TickServer
#else
class FollowerModule : public yarp::os::RFModule
#endif
{
public:

    FollowerModule();
    ~FollowerModule();

    double getPeriod();

    bool updateModule();

    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();

    #ifdef TICK_SERVER
    ReturnStatus request_tick(const std::string& params = "") override;
    ReturnStatus request_status() override;
    ReturnStatus request_halt(const std::string& params = "") override;
    #endif

private:

    FollowerTarget::Follower m_follower;
    double m_period;

    Result_t m_followerResult;

    FollowerTarget::TargetType_t         m_targetType;
    std::unique_ptr<FollowerTarget::TargetRetriever> m_pointRetriever_ptr;

    yarp::os::Port m_rpcPort;
    #ifdef TICK_SERVER
    yarp::os::Port m_toMonitorPort;
    #endif

    StatInfo m_statInfo;
    bool m_useDurationStatInfo;

    #ifdef TICK_SERVER
    BTMonitor::Monitor m_monitor;
    void processTransition(FollowerSMTransition t);
    #endif
};
}//end namespace

