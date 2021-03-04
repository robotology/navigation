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

YARP_LOG_COMPONENT(NAVIGATION_GUI_MAIN, "navigation.navigationGui.main")

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
            yCError(NAVIGATION_GUI_MAIN) << "Unable to open module ports";
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
            if (guiThread) guiThread->stop();
            return false;
        }

        int loc, las, sta;
        if (guiThread) guiThread->getTimeouts(loc, las, sta);

        bool err = false;
        if (las > TIMEOUT_MAX)
        {
            yCError(NAVIGATION_GUI_MAIN,"timeout, no laser data received!\n");
            err = true;
        }
        if (loc > TIMEOUT_MAX)
        {
            yCError(NAVIGATION_GUI_MAIN," timeout, no localization data received!\n");
            err = true;
        }
        if (sta > TIMEOUT_MAX)
        {
            yCError(NAVIGATION_GUI_MAIN,"timeout, no status info received!\n");
            err = true;
        }

        if (err == false)
        {
            std::string status = "error";
            if (guiThread) status = guiThread->getNavigationStatusAsString();
            yCInfo(NAVIGATION_GUI_MAIN) << "module running, ALL ok. Navigation status:" << status;
        }
        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        std::lock_guard<std::mutex> lock(guiThread->m_guithread_mutex);
        if (rpcPort.isOpen() == false) return false;

        reply.clear(); 

        if (command.get(0).isString())
        {
            if (command.get(0).asString()=="quit")
            {
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
                yCDebug(NAVIGATION_GUI_MAIN) << "Not yet implemented.";
            }
        }
        else
        {
            yCError(NAVIGATION_GUI_MAIN) << "Invalid command type";
            reply.addVocab(VOCAB_ERR);
        }
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yCError(NAVIGATION_GUI_MAIN,"check Yarp network.\n");
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

 
