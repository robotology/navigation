/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/IFrameTransform.h>

#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <math.h>

#include "freeFloorViewer.h"

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
    rf.setDefaultConfigFile("freeFloorViewer_sim.ini");           //overridden by --from parameter
    rf.setDefaultContext("freeFloorViewer");                  //overridden by --context parameter
    rf.configure(argc,argv);
    //std::string debug_rf = rf.toString();
    FreeFloorViewer currViewer;

    return currViewer.runModule(rf);
}
