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

#include <string>
#include <yarp/dev/INavigation2D.h>

#ifndef PATH_PLANNER_HELPERS
#define PATH_PLANNER_HELPERS

namespace pathPlannerHelpers
{
    //converts a string to a NavigationStatusEnum.
    //navigation_status_error is returned if the string is not recognized.
    yarp::dev::NavigationStatusEnum string2status(std::string s);
 
    //converts a NavigationStatusEnum to a string.
    std::string getStatusAsString(yarp::dev::NavigationStatusEnum status);
}

#endif