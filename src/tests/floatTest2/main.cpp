/*
•   Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
•   All rights reserved.
•
•   This software may be modified and distributed under the terms of the
•   GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/INavigation2D.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <yarp/rosmsg/std_msgs/Float64Array.h>
#include <yarp/rosmsg/std_msgs/Float32Array.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

//---------------------------------------------------------------------------------------------
void test1_ros_to_yarp()
{
   yDebug() << "\n\ntest1";
   yarp::rosmsg::std_msgs::Float32Array src; src.data.push_back(3); src.data.push_back(4);
   yarp::os::Bottle dst;
   bool b = yarp::os::Portable::copyPortable(src, dst);
   yDebug() << dst.toString();
   yDebug() << b; //this is ok!
}

//---------------------------------------------------------------------------------------------
void test2_yarp64_to_ros64()
{
   yDebug() << "\n\ntest2";
    yarp::os::Bottle src;
    yarp::os::Bottle& src2 = src.addList();
    src2.addFloat64(6464.1);
    src2.addFloat64(6464.2);

    yarp::rosmsg::std_msgs::Float64Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b;                
    yDebug() << src.toString();   
    yDebug() << src2.toString();   
    //yDebug() << dst.toString(); 
}

//---------------------------------------------------------------------------------------------
void test3_yarp64_to_ros32() 
{
    yDebug() << "\n\ntest3";
    yarp::os::Bottle src;
    yarp::os::Bottle& src2 = src.addList();
    src2.addFloat64(64.1);
    src2.addFloat64(64.2);

    yarp::rosmsg::std_msgs::Float32Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b;                
    yDebug() << src.toString();   
    //yDebug() << dst.toString(); Ciao jhvyhgyjjyghche ugyhugiyuhhuhiuiuhhuhbello
}

//---------------------------------------------------------------------------------------------
/*void test4a_yarp32_to_ros32() 
{
    yDebug() << "\n\ntest4a";
    yarp::os::Bottle src;
    src.addFloat32(3232.1);
    src.addFloat32(3232.1);

    yarp::rosmsg::std_msgs::Float32Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b;                
    yDebug() << src.toString();   
    //yDebug() << dst.toString();
}*/

void test4b_yarp32_to_ros32()
{
    yDebug() << "\n\ntest4b";
    yarp::os::Bottle src;
    yarp::os::Bottle& src2 = src.addList();
    src2.addFloat32(3232.1);
    src2.addFloat32(3232.2);
    src2.addFloat32(std::numeric_limits<float>::infinity());

    yarp::rosmsg::std_msgs::Float32Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b;
    yDebug() << src.toString();
    //yDebug() << dst.toString();
}

//---------------------------------------------------------------------------------------------
void test5_yarp32_to_ros64() 
{
    yDebug() << "\n\ntest5";
    yarp::os::Bottle src;
    yarp::os::Bottle& src2 = src.addList();
    src2.addFloat32(64.1);
    src2.addFloat32(64.2);
 //   src2.addFloat32(std::numeric_limits<float>::infinity());

    yarp::rosmsg::std_msgs::Float64Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b;               
    yDebug() << src.toString();   
    //yDebug() << dst.toString();
}

//---------------------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    test1_ros_to_yarp(); //this is ok
    test2_yarp64_to_ros64(); //ok
    test3_yarp64_to_ros32(); //-------
  //  test4a_yarp32_to_ros32(); //sbagliato volontariamente
    test4b_yarp32_to_ros32(); //
    test5_yarp32_to_ros64(); //----------
}
