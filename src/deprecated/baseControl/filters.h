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
    /**
    * These functions implement a butterworth low pass first order, with a cut off frequency of 0.5-8Hz.
    * Coefficients have been computed in order to be used with a sampling frequency of 50Hz (20ms)
    * @param input the value to be filtered
    * @param i the joint number (0-9)
    * @return the filtered value
    */
    double lp_filter_8Hz    (double input, int i);
    double lp_filter_4Hz    (double input, int i);
    double lp_filter_2Hz    (double input, int i);
    double lp_filter_1Hz    (double input, int i);
    double lp_filter_0_5Hz  (double input, int i);

    /**
    * This function implements a rate limiter filter.
    * @param input the value to be filtered
    * @param i the joint number
    * @param rate the maximum change rate
    * @return the filtered value
    */
    double ratelim_filter_0 (double input, int i, double rate_pos, double rate_neg);
}
#endif
