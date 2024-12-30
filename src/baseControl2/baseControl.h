/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "controlThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class CtrlModule: public RFModule
{
protected:
    ControlThread  *control_thr=nullptr;
    Port            rpcPort;
    bool            verbose_print=true;

public:
    CtrlModule() {};
    bool   configure(ResourceFinder &rf) override;
    bool   respond(const Bottle& command, Bottle& reply);
    bool   close();
    double getPeriod();
    bool   updateModule();
};
