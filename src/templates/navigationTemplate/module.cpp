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
#include "module.h"

navigationModule::navigationModule()
{
    navigation_status=yarp::dev::navigation_status_idle;
    iLoc=0;
}

bool navigationModule::configure(yarp::os::ResourceFinder &rf)
{
    //open the rpc port
    bool ret = rpcPort.open("/navigationModule/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }
    attach(rpcPort);

    //open the localization driver
    yarp::os::Property loc_options;
    loc_options.put("device", "localization2DClient");
    loc_options.put("local", "/robotGoto/localizationClient");
    loc_options.put("remote", "/localizationServer");
    loc_options.put("period", 10);

    if (pLoc.open(loc_options) == false)
    {
        yError() << "Unable to open localization driver";
        return false;
    }
    pLoc.view(iLoc);
    if (iLoc==0)
    {
        yError() << "Unable to open localization interface";
        return false;
    }

    return true;
}

bool navigationModule::interruptModule()
{
    rpcPort.interrupt();
    return true;
}

bool navigationModule::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}

double navigationModule::getPeriod()
{ 
    return 1.0; 
}
    
bool navigationModule::updateModule()
{ 
    if (isStopping())
    {
        return false;
    }
    return true; 
}

bool navigationModule::respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
{
    reply.clear();

    if (command.get(0).isVocab())
    {
        if(command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
        {
            int request = command.get(1).asVocab();
            if (request == VOCAB_NAV_GOTOABS)
            {
                yarp::dev::Map2DLocation loc;
                loc.map_id = command.get(2).asString();
                loc.x = command.get(3).asDouble();
                loc.y = command.get(4).asDouble();
                loc.theta = command.get(5).asDouble();
                this->setNewAbsTarget(loc);
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GOTOREL)
            {
                yarp::sig::Vector v;
                v.push_back(command.get(2).asDouble());
                v.push_back(command.get(3).asDouble());
                if (command.size() == 5) v.push_back(command.get(4).asDouble());
                this->setNewRelTarget(v);
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GET_STATUS)
            {
                int nav_status = (int) this->getNavigationStatus();
                reply.addVocab(VOCAB_OK);
                reply.addInt(nav_status);
            }
            else if (request == VOCAB_NAV_STOP)
            {
                this->stopMovement();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_SUSPEND)
            {
                double time = -1;
                if (command.size() > 1)
                    time = command.get(1).asDouble();
                this->pauseMovement(time);
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_RESUME)
            {
                this->resumeMovement();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GET_CURRENT_POS)
            {
                yarp::dev::Map2DLocation loc;
                this->getCurrentPos(loc);
                reply.addVocab(VOCAB_OK);
                reply.addString(loc.map_id);
                reply.addDouble(loc.x);
                reply.addDouble(loc.y);
                reply.addDouble(loc.theta);
            }
            else if (request == VOCAB_NAV_GET_ABS_TARGET || request == VOCAB_NAV_GET_REL_TARGET)
            {
                yarp::dev::Map2DLocation loc;
                if (request == VOCAB_NAV_GET_ABS_TARGET)
                {
                    this->getCurrentAbsTarget(loc);
                }
                else
                {
                    this->getCurrentRelTarget(loc);
                }
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
        else
        {
            yError() << "Invalid vocab received";
            reply.addVocab(VOCAB_ERR);
        }
    }
    else if (command.get(0).isString())
    {
        if (command.get(0).asString()=="quit")
        {
            return false;
        }

        else if (command.get(0).asString()=="help")
        {
            //### TO BE IMPLEMENTED BY USER
            reply.addVocab(yarp::os::Vocab::encode("many"));
            reply.addString("Put here module help");
        }
    }
    else
    {
        yError() << "Invalid command type";
        reply.addVocab(VOCAB_ERR);
    }
    return true;
}

bool navigationModule::setNewAbsTarget(yarp::dev::Map2DLocation loc)
{
    if (navigation_status==yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

bool navigationModule::setNewRelTarget(yarp::sig::Vector v)
{
    if (navigation_status==yarp::dev::navigation_status_idle)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "A navigation task is already running. Stop it first";
    return false;
}

yarp::dev::NavigationStatusEnum navigationModule::getNavigationStatus()
{
    return navigation_status;
}

bool navigationModule::stopMovement()
{
    //### TO BE IMPLEMENTED BY USER
    navigation_status=yarp::dev::navigation_status_idle;
    return true;
}

bool navigationModule::getCurrentAbsTarget(yarp::dev::Map2DLocation& target)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    target = curr;
    return true;
}

bool navigationModule::getCurrentRelTarget(yarp::dev::Map2DLocation& target)
{
    //### TO BE IMPLEMENTED BY USER
    yarp::dev::Map2DLocation curr;
    target = curr;
    return true;
}

bool navigationModule::pauseMovement(double time)
{
    if (navigation_status==yarp::dev::navigation_status_moving)
    {
        //### TO BE IMPLEMENTED BY USER
        navigation_status=yarp::dev::navigation_status_paused;
        return true;
    }
    yError() << "Unable to pause current navigation task";
    return false;
}

bool navigationModule::resumeMovement()
{
    if (navigation_status==yarp::dev::navigation_status_paused)
    {
        //### TO BE IMPLEMENTED BY USER
        return true;
    }
    yError() << "Unable to resume any paused navigation task";
    return false;
}

bool navigationModule::getCurrentPos(yarp::dev::Map2DLocation& loc)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}
