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

#ifndef MAP_H
#define MAP_H

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <cv.h>
#include <highgui.h> 
#include <queue>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

#ifndef M_PI
#define M_PI 3.14159265
#endif

//draw stuff on the map
void drawPath(IplImage *map, MapGrid2D::XYCell current_position, MapGrid2D::XYCell current_target, std::queue<MapGrid2D::XYCell> path, const CvScalar& color);
void drawCurrentPosition(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color);
void drawLaserScan(IplImage *map, std::vector <MapGrid2D::XYCell>& laser_scan, const CvScalar& color);
bool sendToPort(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send);

//return true if the straight line that connects src with dst does not contain any obstacles
bool checkStraightLine(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell src, MapGrid2D::XYCell dst);

//simplify the path
bool simplifyPath(yarp::dev::MapGrid2D& map, std::queue<MapGrid2D::XYCell> input_path, std::queue<MapGrid2D::XYCell>& output_path);

//compute the path
bool findPath(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell start, MapGrid2D::XYCell goal, std::queue<MapGrid2D::XYCell>& path);
#endif
