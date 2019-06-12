/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

/**
 * \section navigationGUI
 * To be written.
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include "navGui.h"
#include <math.h>

class NavigationGUI : public yarp::os::RFModule
{
protected:
    NavGuiThread *guiThread;
    yarp::os::Port     rpcPort;

public:
    NavigationGUI()
    {
        guiThread =NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        guiThread = new NavGuiThread(0.020,rf);

        bool ret = rpcPort.open("/navigationGUI/rpc");
        if (ret == false)
        {
            yError() << "Unable to open module ports";
            return false;
        }
        attach(rpcPort);
        //attachTerminal();

        if (!guiThread->start())
        {
            delete guiThread;
            return false;
        }

        return true;
    }

    virtual bool interruptModule()
    {
        rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        guiThread->stop();
        delete guiThread;
        guiThread =NULL;

        return true;
    }

    virtual double getPeriod()
    { 
        return 3.0; 
    }
    
    virtual bool updateModule()
    { 
        if (isStopping())
        {
            guiThread->stop();
            return false;
        }
        
        int loc, las, sta;
        guiThread->getTimeouts(loc,las,sta);

        bool err = false;
        if (las>TIMEOUT_MAX)
        {
            yError("timeout, no laser data received!\n");
            err= true;
        }
        if (loc>TIMEOUT_MAX)
        {
            yError(" timeout, no localization data received!\n");
            err= true;
        }
        if (sta>TIMEOUT_MAX)
        {
            yError("timeout, no status info received!\n");
            err= true;
        }
        
        if (err==false)
            yInfo() << "module running, ALL ok. Navigation status:" << guiThread->getNavigationStatusAsString();

        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        reply.clear(); 

        guiThread->m_mutex.wait();
        if (command.get(0).isString())
        {
            if (command.get(0).asString()=="quit")
            {
                guiThread->m_mutex.post();
                return false;
            }

            else if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("quit");
                reply.addString("draw_locations <0/1>");
            }
            else if (command.get(0).isString())
            {
                yDebug() << "Not yet implemented.";
            }
        }
        else
        {
            yError() << "Invalid command type";
            reply.addVocab(VOCAB_ERR);
        }
        guiThread->m_mutex.post();
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("navigationGUI.ini");           //overridden by --from parameter
    rf.setDefaultContext("navigationGUI");                  //overridden by --context parameter
    rf.configure(argc,argv);
    std::string debug_rf = rf.toString();

    NavigationGUI navGui;

    return navGui.runModule(rf);
}

 
