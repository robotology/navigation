/* 
 * Copyright (C)2017 ICub Facility - Istituto Italiano di Tecnologia
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
#include <yarp/dev/INavigation2D.h>
#include <math.h>

#ifndef NAV_MODULE_H
#define NAV_MODULE_H

class localizationModule : public yarp::os::RFModule
{
protected:
    yarp::os::Port     rpcPort;

public:
    /**
    * Default module constructor
    */
    localizationModule();

    //declared in yarp::os::RFModule
public:
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool interruptModule();
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply);

public:
    /**
    * Initializes the localization algorithm with the given location.
    * @param loc the initial guess for the robot location
    * @return true/false if the command is accepted
    */
    bool initializeLocalization(yarp::dev::Map2DLocation loc);

    /**
    * Get the current position of the robot, estimated by the localization algorithm
    * @param loc the current position of the robot
    * @return true/false if the command is accepted
    */
    bool getLocalizationData(yarp::dev::Map2DLocation& loc);
};

#endif
