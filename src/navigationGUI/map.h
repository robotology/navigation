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

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/dev/MapGrid2D.h>
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

//! Helper functions which operates on a map grid, computing a path, drawing an image etc.
namespace map_utilites
{
    //draw stuff on the map image
    bool drawInfo(IplImage *map, MapGrid2D::XYCell current, MapGrid2D::XYCell orig, MapGrid2D::XYCell x_axis, MapGrid2D::XYCell y_axis, std::string status, const yarp::dev::Map2DLocation& localiz, const CvFont& font, const CvScalar& color);
    bool drawPath(IplImage *map, MapGrid2D::XYCell current_position, MapGrid2D::XYCell current_target, std::queue<MapGrid2D::XYCell> path, const CvScalar& color);
    bool drawCurrentPosition(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color);
    bool drawGoal(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color);
    bool drawLaserScan(IplImage *map, std::vector <MapGrid2D::XYCell>& laser_scan, const CvScalar& color);
    bool drawLaserMap(IplImage *map, const yarp::dev::MapGrid2D& laserMap, const CvScalar& color);
    bool drawPose(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color);

    //sends and image through the given port
    bool sendToPort(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send);

    // register new obstacles into a map
    void update_obstacles_map(yarp::dev::MapGrid2D& map_to_be_updated, const yarp::dev::MapGrid2D& obstacles_map);
};

#endif
