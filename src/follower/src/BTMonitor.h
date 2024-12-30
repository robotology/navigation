/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file BTMonitor.h
 * @authors: Valentina Gaggero <valentina.gaggero@iit.it>
 */
#include <yarp/os/Port.h>

namespace FollowerTarget
{
    namespace BTMonitor
    {
        enum class Event
        {
            e_from_bt,
            e_req_first_target,
            e_reply_first_target_invalid,
            e_req_target,
            e_reply_N_target_invalid,
            e_req_lookUp,
            e_reply_lookup_failed,
            e_reply_lookup_succeed,
            e_req_navig,
            e_reply_human_lost,
            e_reply_target_found,
            e_req_help,
            e_timeout,
            e_req_update_target,
            e_reply_update_failed,
            e_req

        };



        class Monitor
        {

        public:

            bool init(void);

            void sendEvent(Event e);
        private:

            std::map<Event, std::string> mapEvt2Str;
            bool m_isRunning;
            yarp::os::Port m_toMonitorPort;

        };
    }
}
