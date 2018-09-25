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

localizationModule::localizationModule()
{

}

bool localizationModule::configure(yarp::os::ResourceFinder &rf)
{
    bool ret = rpcPort.open("/localizationModule/rpc");
    if (ret == false)
    {
        yError() << "Unable to open module ports";
        return false;
    }
    attach(rpcPort);
    return true;
}

bool localizationModule::interruptModule()
{
    rpcPort.interrupt();
    return true;
}

bool localizationModule::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    return true;
}

double localizationModule::getPeriod()
{
    return 1.0; 
}

bool localizationModule::updateModule()
{
    if (isStopping())
    {
        return false;
    }
    return true; 
}

bool localizationModule::initializeLocalization(yarp::dev::Map2DLocation loc)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool localizationModule::getLocalizationData(yarp::dev::Map2DLocation& loc)
{
    //### TO BE IMPLEMENTED BY USER
    return true;
}

bool localizationModule::respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).isVocab())
    {
        if(command.get(0).asVocab() == VOCAB_INAVIGATION && command.get(1).isVocab())
        {
            int request = command.get(1).asVocab();
            if (request == VOCAB_NAV_GET_CURRENT_POS)
            {
                reply.addVocab(VOCAB_OK);
                yarp::dev::Map2DLocation loc;
                this->getLocalizationData(loc);
                reply.addString(loc.map_id);
                reply.addDouble(loc.x);
                reply.addDouble(loc.y);
                reply.addDouble(loc.theta);
            }
            else if (request == VOCAB_NAV_SET_INITIAL_POS)
            {
                yarp::dev::Map2DLocation loc;
                loc.map_id = command.get(2).asString();
                loc.x = command.get(3).asDouble();
                loc.y = command.get(4).asDouble();
                loc.theta = command.get(5).asDouble();
                this->initializeLocalization(loc);
                reply.addVocab(VOCAB_OK);
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
