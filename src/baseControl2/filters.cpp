/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "filters.h"
#include "yarp/os/Log.h"
#include "yarp/os/LogStream.h"

double control_filters::lp_filter_8Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 2Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input / 2.818993247e+00;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.2905268567   * yv[0][i]);
    return yv[1][i];
}

double control_filters::lp_filter_4Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 4Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /4.894742855e+00;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.5913983514  * yv[0][i]);
    return yv[1][i];
}

double control_filters::lp_filter_2Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 2Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /8.915815088e+00;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.7756795110 * yv[0][i]);
    return yv[1][i];
}

double control_filters::lp_filter_1Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 1Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /1.689454484e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.8816185924 * yv[0][i]);
    return yv[1][i];
}

double control_filters::lp_filter_0_5Hz(double input, int i)
{
    //This is a butterworth low pass first order, with a cut off frequency of 0.5Hz
    //It must be used with a sampling frequency of 50Hz (20ms)
    static double xv[2][10], yv[2][10];
    xv[0][i] = xv[1][i]; 
    xv[1][i] = input /3.282051595e+01;
    yv[0][i] = yv[1][i]; 
    yv[1][i] =   (xv[0][i] + xv[1][i]) + (  0.9390625058 * yv[0][i]);
    return yv[1][i];
}

double control_filters::ratelim_filter_0(double input, int i, double rate_pos, double rate_neg, double max_val, double min_val)
{
    //This is a rate limiter filter. 
    static double prev[10];

//    if (i==9) {yCDebug() << "9>>>" << input << rate_pos << rate_neg << prev[9];} //print lin vel along x axis
    
    if (prev[i]>max_val) prev[i]=max_val;
    if (prev[i]<min_val) prev[i]=min_val;
    
    if (input*prev[i]>=0)
    {
        if (input > 0)
        {
            if (abs(input) > abs(prev[i]))
            {
                if (abs(input - prev[i]) > rate_pos) prev[i] = prev[i] + rate_pos;
                else     prev[i] = input;
                return prev[i];
            }
            else
            {
                if (abs(input - prev[i]) > rate_neg) prev[i] = prev[i] - rate_neg;
                else     prev[i] = input;
                return prev[i];
            }
        }
        if (input < 0)
        {
        
            if (abs(input) > abs(prev[i]))
            {
                if (abs(input - prev[i]) > rate_pos) prev[i] = prev[i] - rate_pos;
                else     prev[i] = input;
                return prev[i];
            }
            else
            {
                if (abs(input - prev[i]) > rate_neg) prev[i] = prev[i] + rate_neg;
                else     prev[i] = input;
                return prev[i];
            }
        }
        if (input == 0)
        {
            if (prev[i]>0)
            {
                if (abs(input - prev[i]) > rate_neg) prev[i] = prev[i] - rate_neg;
                else     prev[i] = input;
                return prev[i];
            }
            else
            {
                if (abs(input - prev[i]) > rate_neg) prev[i] = prev[i] + rate_neg;
                else     prev[i] = input;
                return prev[i];
            }
        }
    }
    else
    {
        if (input > 0)
        {

            if (abs(input - prev[i]) > rate_pos) prev[i] = prev[i] + rate_pos;
            else     prev[i] = input;
            return prev[i];
        }
        else
        {
            if (abs(input - prev[i]) > rate_neg) prev[i] = prev[i] - rate_neg;
            else     prev[i] = input;
            return prev[i];
        }
    }
}
