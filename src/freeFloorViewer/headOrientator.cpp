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

#include "headOrientator.h"

YARP_LOG_COMPONENT(HEAD_ORIENTATOR, "navigation.headOrientator")

bool HeadOrientator::configure(yarp::os::Property rf)
{
    // --------- Generic config --------- //
    if(rf.check("img_width")){m_imgWidth = rf.find("img_width").asInt();}
    else
    {
        yCError(HEAD_ORIENTATOR,"Img_width not found");
        return false;
    }
    if(rf.check("img_height")) {m_imgHeight = rf.find("img_height").asInt();}
    else
    {
        yCError(HEAD_ORIENTATOR,"Img_height not found");
        return false;
    }
    if(rf.check("hor_fov")) {m_horFOV = rf.find("hor_fov").asDouble();}
    else
    {
        yCError(HEAD_ORIENTATOR,"Hor_fov not found");
        return false;
    }
    if(rf.check("ver_fov")) {m_verFOV = rf.find("ver_fov").asDouble();}
    else
    {
        yCError(HEAD_ORIENTATOR,"Ver_fov not found");
        return false;
    }
    m_vToPitch = m_verFOV/m_imgHeight;
    m_uToYaw = m_horFOV/m_imgWidth;

    // ----------- Polydriver config ----------- //
    yarp::os::Property polyProp;
    polyProp.fromString(rf.toString());
    m_neck.open(polyProp);
    if(!m_neck.isValid())
    {
        yCError(HEAD_ORIENTATOR,"Error opening PolyDriver check parameters");
        return false;
    }

    // --------- IPositionControl device config --------- //
    m_neck.view(m_iNeckPos);
    if(!m_iNeckPos)
    {
        yCError(HEAD_ORIENTATOR,"Error opening iPositionControl interface. Device not available");
        return false;
    }

    // --------- IControlLimits device config --------- //
    m_neck.view(m_iNeckLimits);
    if(!m_iNeckLimits)
    {
        yCError(HEAD_ORIENTATOR,"Error opening iControlimits interface. Device not available");
        return false;
    }

    return true;
}

void HeadOrientator::onRead(yarp::os::Bottle &b)
{
    bool *done = new bool[1];
    m_iNeckPos->checkMotionDone(done);
    if(!done[0])
    {
        m_iNeckPos->stop(0);
        m_iNeckPos->stop(1);

        delete[] done;

        return;
    }

    if(b.size()==2)
    {
        backToZero();
    }
    else if(b.size()==4)
    {
        rotateHead(b);
    }
    else{
        yCError(HEAD_ORIENTATOR,"Wrong bottle received");
    }

    delete[] done;
}

void HeadOrientator::backToZero()
{
    m_iNeckPos->positionMove(0,0.0);
    m_iNeckPos->positionMove(1,0.0);
}

void HeadOrientator::rotateHead(yarp::os::Bottle &b)
{
    double dX = b.get(0).asDouble() - b.get(2).asDouble();
    double dY = b.get(3).asDouble() - b.get(1).asDouble();

    m_iNeckPos->relativeMove(1,dX*m_uToYaw);
    m_iNeckPos->relativeMove(0,dY*m_vToPitch);
}

void HeadOrientator::close()
{
    if(m_neck.isValid())
    {
        m_neck.close();
    }
}
