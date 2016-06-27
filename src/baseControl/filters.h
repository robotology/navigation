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

#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

namespace control_filters
{
    double lp_filter_8Hz    (double input, int i);
    double lp_filter_4Hz    (double input, int i);
    double lp_filter_2Hz    (double input, int i);
    double lp_filter_1Hz    (double input, int i);
    double lp_filter_0_5Hz  (double input, int i);
    double ratelim_filter_0 (double input, int i, double rate);
}
#endif
