/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file NavigationController.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#include <yarp/os/Property.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include "NavigationController.h"



using namespace FollowerTarget;
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

bool NavigationController::configure(yarp::os::ResourceFinder &rf)
{
    bool            okClient, okView, enabled;
    Property        navClientCfg, pLocationServer_cfg;
    string          navServerRoot;

    if(rf.check("autonomousNavEnabled"))
    {
        enabled=rf.find("autonomousNavEnabled").asBool();
    }
    else
    {
        enabled=true;
    }


    if(enabled==false)
    {
        yDebug() << "Autonomous Navigation is NOT enabled in config file!!!";
        return true;
    }

    yDebug() << "Autonomous Navigation is ENABLED in config file!!!";


    if(rf.check("navServerRoot"))
    {
        navServerRoot = rf.find("navServerRoot").asString();
    }
    else
    {
        navServerRoot = "/navigationServer";
    }
//     pLocationServer_cfg.put("device", "locationsServer");
//     pLocationServer_cfg.put("local", "/locationServer");
//     pLocationServer_cfg.put("ROS_enabled", "");
//     if(!m_locServer_driver.open(pLocationServer_cfg))
//     {
//         yError() << "Error opening location server driver";
//         return false;
//     }

    navClientCfg.put("device",         "navigation2DClient");
    navClientCfg.put("local",          "/follower/nav");
    navClientCfg.put("navigation_server", navServerRoot);
    navClientCfg.put("map_locations_server", "/mapServer");
    navClientCfg.put("localization_server", "/localizationServer");

    m_iNav = nullptr;
    if(!m_navClient_driver.open(navClientCfg))
    {
        yError() << "Error opening navigation client driver";
        return false;
    }

    if(!m_navClient_driver.view(m_iNav))
    {
        yError() << "Error opening navigation interface";
        return false;
    }

    if(m_debugOn)
        yDebug() << "Navigation controller is configured!";

    return true;
}

bool NavigationController::isConfigured(void)
{
    if(m_iNav != nullptr)
        return true; //if the navigation client is running
    else
        return false;
}


bool NavigationController::startAutonomousNav(double x, double y, double theta)
{
    if(nullptr == m_iNav)
        return false;

    if(m_debugOn)
        yDebug() << "NavCtrl: gotoTargetByRelativeLocation" << x << y <<theta;

    m_navStarted=true;
    return(m_iNav->gotoTargetByRelativeLocation(x,y, theta));

}

yarp::dev::NavigationStatusEnum NavigationController::getNavigationStatus(void)
{
    NavigationStatusEnum status=navigation_status_error ;
    if(nullptr == m_iNav)
        return status;

    m_iNav->getNavigationStatus(status);
    return status;
}


bool NavigationController::AbortAutonomousNav(void)
{
    if(nullptr == m_iNav)
        return false;

    bool ret = true;
    if(m_navStarted)
    {
       ret=m_iNav->stopNavigation();
        if(m_debugOn)
            yDebug() << "NavCtrl: abort autonomous navigation";
    }
    m_navStarted=false;
    return ret;
}
