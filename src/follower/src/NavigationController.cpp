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

YARP_LOG_COMPONENT(FOLLOWER_NAV, "navigation.follower.navigationController")

using namespace FollowerTarget;
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

bool NavigationController::configure(yarp::os::ResourceFinder &rf)
{
    bool            okClient, okView, enabled=true;;
    Property        navClientCfg, pLocationServer_cfg;
    string          navServerRoot;

    Bottle config_group = rf.findGroup("FOLLOWER_GENERAL");
    if(!config_group.isNull())
    {
        if(config_group.check("autonomousNavEnabled"))
        {
            enabled=config_group.find("autonomousNavEnabled").asBool();
        }
    }

    if(enabled==false)
    {
        yCDebug(FOLLOWER_NAV) << "Autonomous Navigation is NOT enabled in config file!!!";
        return true;
    }

    yCDebug(FOLLOWER_NAV) << "Autonomous Navigation is ENABLED in config file!!!";


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
//         yCError() << "Error opening location server driver";
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
        yCError(FOLLOWER_NAV) << "Error opening navigation client driver";
        return false;
    }

    if(!m_navClient_driver.view(m_iNav))
    {
        yCError(FOLLOWER_NAV) << "Error opening navigation interface";
        return false;
    }

    if(m_debugOn)
        yCDebug(FOLLOWER_NAV) << "Navigation controller is configured!";

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
        yCDebug(FOLLOWER_NAV) << "NavCtrl: gotoTargetByRelativeLocation" << x << y <<theta;

    m_navStarted=true;
    return(m_iNav->gotoTargetByRelativeLocation(x,y, theta));

}

NavigationStatusEnum NavigationController::getNavigationStatus(void)
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
            yCDebug(FOLLOWER_NAV) << "NavCtrl: abort autonomous navigation";
    }
    m_navStarted=false;
    return ret;
}
