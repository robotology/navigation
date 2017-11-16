/*
* Copyright (C)2015  iCub Facility - Istituto Italiano di Tecnologia
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

#ifndef SIM_ODOMETRY_H
#define SIM_ODOMETRY_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>
#include "../odometry.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class SIM_Odometry: public Odometry
{
private:
    //encoder variables
    double              encL_offset;
    double              encR_offset;

    double              encL;
    double              encR;

    //measured motor velocity
    double              velL;
    double              velR;

    //estimated motor velocity
    double              velL_est;
    double              velR_est;
    iCub::ctrl::AWLinEstimator      *encvel_estimator;
    iCub::ctrl::AWLinEstimator      *encw_estimator;

    //robot geometry
    double              geom_r;
    double              geom_L;

    yarp::sig::Vector enc;
    yarp::sig::Vector encv;

public:
    SIM_Odometry(unsigned int _period, PolyDriver* _driver);
    virtual ~SIM_Odometry();
    bool reset_odometry();
    bool open(ResourceFinder &_rf, Property &options);
    void compute();
    void printStats();
    double get_vlin_coeff();
    double get_vang_coeff();

};

#endif
