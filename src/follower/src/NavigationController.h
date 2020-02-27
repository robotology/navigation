/******************************************************************************
 *                                                                            *
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file NavigationController.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/os/ResourceFinder.h>

namespace FollowerTarget
{
    class NavigationController
    {
    public:
        NavigationController():m_iNav(nullptr), m_debugOn(true), m_navStarted(false){;}

        bool configure(yarp::os::ResourceFinder &rf);
        bool startAutonomousNav(double x, double y, double theta);
        bool AbortAutonomousNav(void); //if navigation is started abort it
        yarp::dev::Nav2D::NavigationStatusEnum getNavigationStatus(void);
        bool isConfigured(void);
        void setDebug(bool on) {m_debugOn=on;}
    private:
        //device drivers and interfaces
        yarp::dev::PolyDriver      m_navClient_driver;
        yarp::dev::PolyDriver      m_locServer_driver;
        yarp::dev::Nav2D::INavigation2D*  m_iNav; //ATTENTION: this pointer can be null if AutoNav is not configured because .ini file disables  it

        bool m_debugOn;
        bool m_navStarted;
    };

}
