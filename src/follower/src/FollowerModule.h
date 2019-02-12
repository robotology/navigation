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


#include "TargetRetriver.h"
#include "Follower.h"

#ifdef TICK_SERVER
#include <tick_server.h>
#include <ReturnStatus.h>
#endif




#ifdef TICK_SERVER
class FollowerModule:public yarp::os::RFModule, public TickServer
#else
class FollowerModule : public RFModule
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
    ReturnStatus request_halt() override;
    #endif

private:

    FollowerTarget::Follower m_follower;
    double m_period;
    double const m_defaultPeriod=0.01;

    FollowerTarget::FollowerTargetType         m_targetType;
    std::unique_ptr<FollowerTarget::TargetRetriver> m_pointRetriver_ptr;

    yarp::os::Port m_rpcPort;

};


