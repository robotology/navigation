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

void test2()
{
   yarp::rosmsg::std_msgs::Float32Array src; src.data.push_back(3); src.data.push_back(4);
   yarp::os::Bottle dst;
   bool b = yarp::os::Portable::copyPortable(src, dst);
   yDebug() << dst.toString();
   yDebug() << b; //this is ok!
}

void test1()
{
#if 0
    yarp::os::Bottle src;
    src.addFloat64(33.3);
    src.addFloat64(33.3);
#else
    yarp::os::Bottle src;
    yarp::os::Bottle& src2 = src.addList();
    src2.addFloat64(6464.1);
    src2.addFloat64(6464.2);
    src2.addFloat64(std::numeric_limits<double>::infinity());
#endif

    yarp::rosmsg::std_msgs::Float64Array dst;
    bool b = yarp::os::Portable::copyPortable(src, dst);
    yDebug() << b; //always false
}

template <class T>
int send(yarp::os::Bottle& tmp, yarp::os::Contactable* c, string s)
{
    yarp::os::BufferedPort<T>* the_port = dynamic_cast<yarp::os::BufferedPort<T>*> (c);
    if (the_port == nullptr) { yError()<<"dynamic_cast failed" << s; return -1; }

    auto& dat = the_port->prepare();
    if (yarp::os::Portable::copyPortable(tmp, dat)==false)
    { yDebug() << "copyportable failed" << s; }

    the_port->writeStrict();
}

int main(int argc, char* argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    test1();
    test2();

    yarp::os::Contactable* outputPort3232 = new BufferedPort<yarp::rosmsg::std_msgs::Float32Array>;
    yarp::os::Contactable* outputPort6432 = new BufferedPort<yarp::rosmsg::std_msgs::Float32Array>;
    yarp::os::Contactable* outputPort3264 = new BufferedPort<yarp::rosmsg::std_msgs::Float64Array>;
    yarp::os::Contactable* outputPort6464 = new BufferedPort<yarp::rosmsg::std_msgs::Float64Array>;

    outputPort3232->open("/port3232");
    outputPort6432->open("/port6432");
    outputPort3264->open("/port3264");
    outputPort6464->open("/port6464");

    yarp::os::Time::delay(1);

//*******THIS*********
#if 0
    Bottle b64; b64.addFloat64(6464.6464);
    Bottle b32; b32.addFloat32(3232.3232);
#endif
//*******OR THIS?? In think this*********
    Bottle b64; yarp::os::Bottle& bb64 = b64.addList(); bb64.addFloat64(6464.1); bb64.addFloat64(6464.2); bb64.addFloat64(std::numeric_limits<double>::infinity());
    Bottle b32; yarp::os::Bottle& bb32 = b32.addList(); bb32.addFloat32(3232.1); bb32.addFloat32(3232.2); bb32.addFloat32(std::numeric_limits<float>::infinity());

    for (auto i=0; i<1000;i++)
    {
        yDebug();
        send< yarp::rosmsg::std_msgs::Float32Array>(b32, outputPort3232,"3232"); 
        send< yarp::rosmsg::std_msgs::Float32Array>(b64, outputPort6432,"6432"); 
        send< yarp::rosmsg::std_msgs::Float64Array>(b32, outputPort3264,"3264");
        send< yarp::rosmsg::std_msgs::Float64Array>(b64, outputPort6464,"6464");
        yarp::os::Time::delay(1);
    }

    outputPort3232->close();
    outputPort6432->close();
    outputPort3264->close();
    outputPort6464->close();
    delete outputPort3232;
    delete outputPort6432;
    delete outputPort3264;
    delete outputPort6464;
}
