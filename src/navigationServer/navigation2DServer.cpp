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

#define _USE_MATH_DEFINES

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/MapGrid2D.h>
#include <math.h>
#include <cmath>
#include "navigation2DServer.h"

using namespace yarp::os;
using namespace yarp::dev;

#ifndef RAD2DEG
#define RAD2DEG 180.0/M_PI
#endif

navigation2DServer::navigation2DServer() : PeriodicThread(DEFAULT_THREAD_PERIOD)
{
    m_navigation_status=yarp::dev::navigation_status_idle;
}

bool navigation2DServer::attachAll(const PolyDriverList &device2attach)
{
    if (device2attach.size() != 1)
    {
        yError("Navigation2DServer: cannot attach more than one device");
        return false;
    }

    yarp::dev::PolyDriver * Idevice2attach = device2attach[0]->poly;

    if (Idevice2attach->isValid())
    {
        Idevice2attach->view(iNav_target);
        Idevice2attach->view(iNav_ctrl);
    }

    if (nullptr == iNav_target ||
        nullptr == iNav_ctrl)
    {
        yError("Navigation2DServer: subdevice passed to attach method is invalid");
        return false;
    }

    PeriodicThread::setPeriod(m_period);
    return PeriodicThread::start();
}

bool navigation2DServer::detachAll()
{
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }
    iNav_target = nullptr;
    iNav_ctrl = nullptr;
    return true;
}

bool navigation2DServer::open(Searchable& config)
{
    Property params;
    params.fromString(config.toString().c_str());

    if (!config.check("period"))
    {
        yError() << "navigation2DServer: missing 'period' parameter. Check you configuration file\n";
        return false;
    }
    else
        m_period = config.find("period").asInt32() / 1000.0;

    if (!config.check("name"))
    {
        yError() << "navigation2DServer: missing 'name' parameter. Check you configuration file; it must be like:";
        yError() << "   name:         full name of the port, like /robotName/deviceId/sensorType:o";
        return false;
    }
    else
    {
        m_rpcPortName = m_streamingPortName + "/rpc:i";
    }

    if (!initialize_YARP(config))
    {
        yError() << "navigation2DServer: Error initializing YARP ports";
        return false;
    }

    if (config.check("subdevice"))
    {
        Property       p;
        PolyDriverList driverlist;
        p.fromString(config.toString(), false);
        p.put("device", config.find("subdevice").asString());

        if (!pNav.open(p) || !pNav.isValid())
        {
            yError() << "navigation2DServer: failed to open subdevice.. check params";
            return false;
        }

        driverlist.push(&pNav, "1");
        if (!attachAll(driverlist))
        {
            yError() << "navigation2DServer: failed to open subdevice.. check params";
            return false;
        }
    }
    return true;
}

bool navigation2DServer::initialize_YARP(yarp::os::Searchable &params)
{
    if (!m_rpcPort.open(m_rpcPortName.c_str()))
    {
        yError("Navigation2DServer: failed to open port %s", m_rpcPortName.c_str());
        return false;
    }
    m_rpcPort.setReader(*this);
    return true;
}

bool navigation2DServer::close()
{
    yTrace("navigation2DServer::Close");
    if (PeriodicThread::isRunning())
    {
        PeriodicThread::stop();
    }

    detachAll();
    return true;
}

bool navigation2DServer::read(yarp::os::ConnectionReader& connection)
{
    yarp::os::Bottle command;
    yarp::os::Bottle reply;
    bool ok = command.read(connection);
    if (!ok) return false;

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
                iNav_target->gotoTargetByAbsoluteLocation(loc);
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GOTOREL)
            {
                if (command.size() == 5)
                {
                    double x = command.get(2).asDouble();
                    double y = command.get(3).asDouble();
                    double theta = command.get(4).asDouble();
                    iNav_target->gotoTargetByRelativeLocation(x,y,theta);
                    reply.addVocab(VOCAB_OK);
                }
                else if (command.size() == 4)
                {
                    double x = command.get(2).asDouble();
                    double y = command.get(3).asDouble();
                    iNav_target->gotoTargetByRelativeLocation(x, y);
                    reply.addVocab(VOCAB_OK);
                }
                else
                {
                    yError() << "Invalid number of params";
                    reply.addVocab(VOCAB_ERR);
                }
            }
            else if (request == VOCAB_NAV_GET_STATUS)
            {
                yarp::dev::NavigationStatusEnum nav_status = yarp::dev::navigation_status_error;
                iNav_ctrl->getNavigationStatus(nav_status);
                reply.addVocab(VOCAB_OK);
                reply.addInt(nav_status);
            }
            else if (request == VOCAB_NAV_STOP)
            {
                iNav_ctrl->stopNavigation();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_SUSPEND)
            {
                double time = -1;
                if (command.size() > 1)
                {
                    time = command.get(1).asDouble();
                    iNav_ctrl->suspendNavigation(time);
                }
                else
                {
                    iNav_ctrl->suspendNavigation();
                }
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_RESUME)
            {
                iNav_ctrl->resumeNavigation();
                reply.addVocab(VOCAB_OK);
            }
            else if (request == VOCAB_NAV_GET_NAVIGATION_WAYPOINTS)
            {
                std::vector<yarp::dev::Map2DLocation> locs;
                bool b = iNav_ctrl->getAllNavigationWaypoints(locs);
                if (b)
                {
                    reply.addVocab(VOCAB_OK);
                    for (size_t i = 0; i < locs.size(); i++)
                    {
                        Bottle waypoint = reply.addList();
                        waypoint.addString(locs[i].map_id);
                        waypoint.addFloat64(locs[i].x);
                        waypoint.addFloat64(locs[i].y);
                        waypoint.addFloat64(locs[i].theta);
                    }
                }
                else
                {
                    //no waypoints available
                    reply.addVocab(VOCAB_OK);
                    reply.addString("invalid");
                }
            }
            else if (request == VOCAB_NAV_GET_CURRENT_WAYPOINT)
            {
                yarp::dev::Map2DLocation loc;
                bool b = iNav_ctrl->getCurrentNavigationWaypoint(loc);
                if (b)
                {
                    reply.addVocab(VOCAB_OK);
                    reply.addString(loc.map_id);
                    reply.addFloat64(loc.x);
                    reply.addFloat64(loc.y);
                    reply.addFloat64(loc.theta);
                }
                else
                {
                    //no waypoint available
                    reply.addVocab(VOCAB_OK);
                    reply.addString("invalid");
                }
            }
            else if (request == VOCAB_GET_NAV_MAP)
            {
                yarp::dev::MapGrid2D map;
                if (iNav_ctrl->getCurrentNavigationMap((yarp::dev::NavigationMapTypeEnum)(command.get(1).asInt()), map))
                {
                    reply.addVocab(VOCAB_OK);
                    yarp::os::Bottle& mapbot = reply.addList();
                    Property::copyPortable(map, mapbot);
                }
                else
                {
                    reply.addVocab(VOCAB_ERR);
                }
            }
            else if (request == VOCAB_NAV_GET_ABS_TARGET || request == VOCAB_NAV_GET_REL_TARGET)
            {
                yarp::dev::Map2DLocation loc;
                if (request == VOCAB_NAV_GET_ABS_TARGET)
                {
                    iNav_target->getAbsoluteLocationOfCurrentTarget(loc);
                }
                else
                {
                    iNav_target->getRelativeLocationOfCurrentTarget(loc.x, loc.y, loc.theta);
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
        }
        else
        {
            yError() << "Invalid vocab received";
            reply.addVocab(VOCAB_ERR);
        }
    }
    else
    {
        yError() << "Invalid command type";
        reply.addVocab(VOCAB_ERR);
    }

    yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
    if (returnToSender != nullptr)
    {
        reply.write(*returnToSender);
    }

    return true;
}

void navigation2DServer::run()
{
}
