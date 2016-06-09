/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef STATUS_H
#define STATUS_H

#include <yarp/os/Vocab.h>
#include <string>

using namespace std;

enum    status_enum {IDLE=0, MOVING, WAITING_OBSTACLE, REACHED, ABORTED, PAUSED, THINKING};

class   status_type
{
    private:
    status_enum internal_status;

    public:
    status_type ()
    {
        internal_status = IDLE;
    }

    int getStatusAsInt()
    {
        return internal_status;
    }

    string      getStatusAsString()
    {
        string s;
        if      (internal_status == IDLE)     s = "IDLE";
        else if (internal_status == MOVING)   s = "MOVING";
        else if (internal_status == WAITING_OBSTACLE)  s = "WAITING_OBSTACLE";
        else if (internal_status == REACHED)  s = "REACHED";
        else if (internal_status == ABORTED)  s = "ABORTED";
        else if (internal_status == PAUSED)   s = "PAUSED";
        else if (internal_status == THINKING) s = "THINKING";
        else 
        {
            printf ("ERROR: unknown status of inner controller!");
            s = "IDLE";
        }
        return s;
    }

    //status_type getStatusAsEnum();
    //status_type getStatusAsVocab();

    status_type& operator=(const status_enum &s)
    {
        this->internal_status=s;
        return *this;
    }

    status_type& operator=(const string &s)
    {
        status_enum status;
        if      (s=="IDLE")     status = IDLE;
        else if (s=="MOVING")   status = MOVING;
        else if (s=="WAITING_OBSTACLE")  status = WAITING_OBSTACLE;
        else if (s=="REACHED")  status = REACHED;
        else if (s=="ABORTED")  status = ABORTED;
        else if (s=="PAUSED")   status = PAUSED;
        else if (s=="THINKING") status = THINKING;
        else 
        {
            printf ("ERROR: unknown status of inner controller!");
            status = IDLE;
        }
        this->internal_status=status;
        return *this;
    }

    bool operator==(const status_type &s) const
    {
        if (s.internal_status==this->internal_status)
            return true;
        else
            return false;
    }

    bool operator==(const status_enum &s) const
    {
        if (s==this->internal_status)
            return true;
        else
            return false;
    }

    bool operator!=(const status_enum &s) const
    {
        if (s!=this->internal_status)
            return true;
        else
            return false;
    }
};

#endif
