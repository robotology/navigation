/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
    double ratelim_filter_0 (double input, int i, double rate_pos, double rate_neg, double max_val, double min_val);
}
#endif
