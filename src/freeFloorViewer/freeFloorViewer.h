/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FREE_FLOOR_VIEWER_H
#define FREE_FLOOR_VIEWER_H

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
#include "freeFloorThread.h"
#include "headOrientator.h"

class FreeFloorViewer : public yarp::os::RFModule
{
protected:
    yarp::os::BufferedPort<yarp::os::Bottle> m_posInputPort;
    yarp::os::BufferedPort<yarp::os::Bottle> m_headInputPort;
    FreeFloorThread*   m_innerThread;
    double m_period;
    HeadOrientator* m_headOrientator;

public:
    FreeFloorViewer();
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual double getPeriod();
    virtual bool updateModule();
};

#endif
