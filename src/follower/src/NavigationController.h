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
        NavigationController():m_iNav(nullptr), m_debugOn(true){;}

        bool configure(yarp::os::ResourceFinder &rf);
        void startAutonomousNav(double x, double y, double theta);
        void AbortAutonomousNav(void);
        yarp::dev::NavigationStatusEnum getNavigationStatus(void);
    private:
        //device drivers and interfaces
        yarp::dev::PolyDriver      m_navClient_driver;
        yarp::dev::PolyDriver      m_locServer_driver;
        yarp::dev::INavigation2D*  m_iNav;

        bool m_debugOn;
    };

}
