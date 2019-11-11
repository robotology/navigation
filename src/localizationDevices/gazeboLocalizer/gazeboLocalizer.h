/*
 * Copyright (C)2018  ICub Facility - Istituto Italiano di Tecnologia
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
#include <yarp/os/Mutex.h>
#include <yarp/os/LockGuard.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Node.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RpcClient.h>
#include <math.h>
#include <yarp/dev/IMap2D.h>

#ifndef GAZEBO_LOCALIZER_H
#define GAZEBO_LOCALIZER_H

using namespace yarp::os;

class gazeboLocalizer;
class gazeboLocalizerThread;

class gazeboLocalizerRPCHandler : public yarp::dev::DeviceResponder
{
protected:
    gazeboLocalizer * interface;
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response) override;

public:
    gazeboLocalizerRPCHandler() : interface(NULL) { }
    void setInterface(gazeboLocalizer* iface);
};

class gazeboLocalizer : public yarp::dev::DeviceDriver,
                        public yarp::dev::ILocalization2D
{
public:
    gazeboLocalizerThread*    thread;
    gazeboLocalizerRPCHandler rpcPortHandler;
    yarp::os::Port            rpcPort;

public:
    virtual bool open(yarp::os::Searchable& config) override;

    gazeboLocalizer();
    virtual ~gazeboLocalizer();

    virtual bool close() override;

public:
    /**
    * Gets the current status of the localization task.
    * @return true/false
    */
    bool   getLocalizationStatus(yarp::dev::LocalizationStatusEnum& status) override;

    /**
    * Gets a set of pose estimates computed by the localization algorithm.
    * @return true/false
    */
    bool   getEstimatedPoses(std::vector<yarp::dev::Nav2D::Map2DLocation>& poses) override;

    /**
    * Gets the current position of the robot w.r.t world reference frame
    * @param loc the location of the robot
    * @return true/false
    */
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc) override;

    /**
    * Sets the initial pose for the localization algorithm which estimates the current position of the robot w.r.t world reference frame.
    * @param loc the location of the robot
    * @return true/false
    */
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc) override;

    /**
     * Gets the current position of the robot w.r.t world reference frame, plus the covariance
     * @param loc the location of the robot
     * @param cov the 3x3 covariance matrix
     * @return true/false
     */
    bool   getCurrentPosition(yarp::dev::Nav2D::Map2DLocation& loc, yarp::sig::Matrix& cov) override;

    /**
    * Sets the initial pose for the localization algorithm which estimates the current position of the robot w.r.t world reference frame.
    * @param loc the location of the robot
    * @param cov the 3x3 covariance matrix
    * @return true/false
    */
    bool   setInitialPose(const yarp::dev::Nav2D::Map2DLocation& loc, const yarp::sig::Matrix& cov) override;

    /**
    * Starts the localization service
    * @return true/false
    */
    bool   startLocalizationService() override;

    /**
    * Stops the localization service
    * @return true/false
    */
    bool   stopLocalizationService() override;
};

class gazeboLocalizerThread : public yarp::os::PeriodicThread
{
protected:
    //general
    double                       m_last_statistics_printed;
    yarp::dev::Nav2D::Map2DLocation     m_map_to_gazebo_transform;
    yarp::dev::Nav2D::Map2DLocation     m_localization_data;
    yarp::dev::Nav2D::Map2DLocation     m_gazebo_data;
    yarp::os::Mutex              m_mutex;
    yarp::os::Searchable&        m_cfg;
    std::string                  m_local_name_prefix;
    std::string                  m_local_gazebo_port_name;
    std::string                  m_remote_gazebo_port_name;
    std::string                  m_object_name;
    yarp::os::RpcClient          m_port_gazebo_comm;


private:
    bool open_gazebo();

public:
    gazeboLocalizerThread(double _period, yarp::os::Searchable& _cfg);
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;

public:
    bool initializeLocalization(const yarp::dev::Nav2D::Map2DLocation& loc);
    bool getCurrentLoc(yarp::dev::Nav2D::Map2DLocation& loc);
};

#endif
