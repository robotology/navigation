/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <yarp/dev/IEncoders.h>

class HeadOrientator : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
private:
    //Polydriver
    yarp::dev::PolyDriver        m_neck;
    yarp::dev::IPositionControl* m_iNeckPos{nullptr};
    yarp::dev::IControlLimits*   m_iNeckLimits{nullptr};
    yarp::dev::IEncoders*        m_iNeckEncoders{nullptr};

    //Parameters
    int    m_imgWidth{0};
    int    m_imgHeight{0};
    double m_horFOV{0.0};
    double m_verFOV{0.0};
    double m_vToPitch{0.0};
    double m_uToYaw{0.0};
    double m_pitchSafeMargin{5.0};
    double m_yawSafeMargin{5.0};
    double m_yawMax{0.0};
    double m_pitchMax{0.0};
    double m_yawMin{0.0};
    double m_pitchMin{0.0};

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
