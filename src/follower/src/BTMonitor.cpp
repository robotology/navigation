/*
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This software may be modified and distributed under the terms of the
* GPL-2+ license. See the accompanying LICENSE file for details.
*/


/**
 * @file BTMonitor.cpp
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */

#ifdef TICK_SERVER
#include <yarp/BT_wrappers/MonitorMsg.h>
#endif

#include "BTMonitor.h"


#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace FollowerTarget;

#define evt2str(e) #e

bool BTMonitor::Monitor::init(void)
{
    mapEvt2Str[Event::e_from_bt] ="e_from_bt";
    mapEvt2Str[Event::e_req_first_target] ="e_req_first_target";
    mapEvt2Str[Event::e_reply_first_target_invalid] ="e_reply_first_target_invalid";
    mapEvt2Str[Event::e_req_target] ="e_req_target";
    mapEvt2Str[Event::e_reply_N_target_invalid] ="e_reply_N_target_invalid";
    mapEvt2Str[Event::e_req_lookUp] ="e_req_lookUp";
    mapEvt2Str[Event::e_reply_lookup_failed] ="e_reply_lookup_failed";
    mapEvt2Str[Event::e_reply_lookup_succeed] ="e_reply_lookup_succeed";
    mapEvt2Str[Event::e_req_navig] ="e_req_navig";
    mapEvt2Str[Event::e_reply_human_lost] ="e_reply_human_lost";
    mapEvt2Str[Event::e_reply_target_found] ="e_reply_target_found";
    mapEvt2Str[Event::e_req_help] ="e_req_help";
    mapEvt2Str[Event::e_timeout] ="e_timeout";
    mapEvt2Str[Event::e_req_update_target] ="e_req_update_target";
    mapEvt2Str[Event::e_reply_update_failed] ="e_reply_update_failed";
    mapEvt2Str[Event::e_req] ="e_req";

    m_isRunning = m_toMonitorPort.open("/follower/monitor:o");
    return m_isRunning;
}


void BTMonitor::Monitor::sendEvent(BTMonitor::Event e)
{
    if(!m_isRunning)
        return;

    yarp::BT_wrappers::MonitorMsg msg;
    msg.skill = "follower";
    msg.event = mapEvt2Str[e];
//    yError() << "SEND EVT= " <<msg.event;
    m_toMonitorPort.write(msg);
}
