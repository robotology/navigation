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

#ifndef HEAD_ORIENTATOR_H
#define HEAD_ORIENTATOR_H

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IControlLimits.h>

class HeadOrientator : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
private:
    //Polydriver
    yarp::dev::PolyDriver m_neck;
    yarp::dev::IPositionControl* m_iNeckPos{nullptr};
    yarp::dev::IControlLimits* m_iNeckLimits{nullptr};

    //Parameters
    int m_imgWidth{0};
    int m_imgHeight{0};
    double m_horFOV{0.0};
    double m_verFOV{0.0};
    double m_vToPitch{0.0};
    double m_uToYaw{0.0};

public:
    //Constructor/Distructor
    HeadOrientator(){}
    ~HeadOrientator(){}

    //Internal methods
    bool configure(yarp::os::Property rf);
    void rotateHead(yarp::os::Bottle& b);
    void backToZero();
    void close();

    //Port callback
    using TypedReaderCallback<yarp::os::Bottle>::onRead;
    void onRead(yarp::os::Bottle& b) override;
};

#endif //HEAD_ORIENTATOR_H
