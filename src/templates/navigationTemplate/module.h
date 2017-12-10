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
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ILocalization2D.h>
#include <math.h>

#ifndef NAV_MODULE_H
#define NAV_MODULE_H

class navigationModule : public yarp::os::RFModule
{
protected:
    yarp::os::Port              rpcPort;
    int                         navigation_status;
    yarp::dev::PolyDriver       pLoc;
    yarp::dev::ILocalization2D* iLoc;

public:
    /**
    * Default module constructor
    */
    navigationModule();

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
    * Sets a new navigation target, expressed in absolute (map) coordinate frame.
    * @param loc the location to be reached
    * @return true/false if the command is accepted
    */
    bool setNewAbsTarget(yarp::dev::Map2DLocation loc);
    
    /**
    * //Sets a new relative target, expressed in local (robot) coordinate frame.
    * @param v a three-element vector (x,y,theta) representing the location to be reached
    * @return true/false if the command is accepted
    */
    bool setNewRelTarget(yarp::sig::Vector v);

    /**
    * //Gets the last target set through a setNewAbsTarget command.
    * @return a Map2DLocation containing data of the current target.
    */
    yarp::dev::Map2DLocation getCurrentAbsTarget();
    
    /**
    * //Gets the last target set through a setNewRelTarget command, expressed in absolute coordinates.
    * @return a Map2DLocation containing data of the current target.
    */
    yarp::dev::Map2DLocation getCurrentRelTarget();
    
    /**
    * //Gets the status of the current navigation task. Typically stored into navigation_status variable.
    * @return the current navigation status expressed as int.
    */
    int getNavigationStatusAsInt();
    
    /**
    * //Stops the current navigation task.
    * @return true/false if the command is executed succesfully.
    */
    bool stopMovement();
    
    /**
    * //Pauses the current navigation task.
    * @return true/false if the command is executed succesfully.
    */
    bool pauseMovement(double time);
    
    /**
    * //Resumes a previosuly paused navigation task.
    * @return true/false if the command is executed succesfully.
    */
    bool resumeMovement();
    
    /**
    * //Gets the current robot position. Typically obtained through the ILocalization2D interface.
    * @param loc the current robot location.
    * @return true/false if the command is executed succesfully.
    */
    bool getCurrentPos(yarp::dev::Map2DLocation& loc);

};

#endif
