/*
 * SPDX-FileCopyrightText: 2026-2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

struct yReturnValue {
} (
  yarp.name = "yarp::dev::ReturnValue"
  yarp.includefile = "yarp/dev/ReturnValue.h"
)

struct return_GetNavigationStatus
{
    1: yReturnValue ret;
    2: string status;
}

//-------------------------------------------------

service robotGotoMsgs
{
    yReturnValue ResetParams ();
    yReturnValue Approach(1: double dir, 2:double speed, 3:double time);
    yReturnValue SetLinearTolerance (1: double tol);
    yReturnValue SetAngularTolerance (1: double tol);
    yReturnValue SetMaxLinearSpeed (1: double maxLin);
    yReturnValue SetMaxAngularSpeed (1: double maxAng);
    yReturnValue SetMinLinearSpeed (1: double minLin);
    yReturnValue SetMinAngularSpeed (1: double minAng);
    yReturnValue SetLinearSpeedGain (1: double LinGain);
    yReturnValue SetAngularSpeedGain (1: double AngGain);
    yReturnValue SetObstacoleAvoidance (1: bool enable);
    yReturnValue SetObstacoleStop (1: bool enable);
    return_GetNavigationStatus   GetNavigationStatus ();
}
