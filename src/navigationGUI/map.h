/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
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
#include <opencv2/imgproc/imgproc_c.h>
#include <queue>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#ifndef M_PI
#define M_PI 3.14159265
#endif

//! Helper functions which operates on a map grid, computing a path, drawing an image etc.
namespace map_utilites
{
    //draw stuff on the map image
    bool drawInfo(IplImage *map, XYCell current, XYCell orig, XYCell x_axis, XYCell y_axis, std::string status, const yarp::dev::Nav2D::Map2DLocation& localiz, const CvFont& font, const CvScalar& color);
    bool drawInfoFixed(IplImage *map, XYCell whereToDraw, XYCell orig, XYCell x_axis, Nav2D::XYCell y_axis, std::string status, const yarp::dev::Nav2D::Map2DLocation& localiz, const CvFont& font, const CvScalar& color);
    bool drawPath(IplImage *map, XYCell current_position, XYCell current_target, std::queue<Nav2D::XYCell> path, const CvScalar& color, const CvScalar& color2);
    bool drawCurrentPosition(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info ,const yarp::dev::Nav2D::Map2DLocation& current, const CvScalar& color);
    bool drawGoal(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info, const yarp::dev::Nav2D::Map2DLocation current, const CvScalar& color);
    bool drawPose(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info, const yarp::dev::Nav2D::Map2DLocation current, const CvScalar& color);
    bool drawArea(IplImage *map, std::vector<XYCell> area, const CvScalar& color);
    bool drawLaserScan(IplImage *map, std::vector <Nav2D::XYCell>& laser_scan, const CvScalar& color);
    bool drawLaserMap(IplImage *map, const yarp::dev::Nav2D::MapGrid2D& laserMap, const CvScalar& color);

    //sends and image through the given port
    bool sendToPort(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send);

    // register new obstacles into a map
    void update_obstacles_map(yarp::dev::Nav2D::MapGrid2D& map_to_be_updated, const yarp::dev::Nav2D::MapGrid2D& obstacles_map);
};

#endif
