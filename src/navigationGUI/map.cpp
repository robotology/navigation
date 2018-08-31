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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/Map2DLocation.h>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h> 

#include "map.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace map_utilites;

bool map_utilites::sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send)
{
    if (port!=0 && port->getOutputCount()>0)
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *segImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
        segImg->resize(image_to_send->width, image_to_send->height );
        cvCopy(image_to_send, (IplImage*)segImg->getIplImage());
        port->prepare() = *segImg;
        port->write();
        delete segImg;
        return true;
    }
    return false;
}

bool map_utilites::drawPath(IplImage *map, MapGrid2D::XYCell current_position, MapGrid2D::XYCell current_target, std::queue<MapGrid2D::XYCell> path, const CvScalar& color)
{
    if (map==0) return false;
    cvLine(map, cvPoint(current_position.x, current_position.y), cvPoint(current_target.x, current_target.y), color);

    if (path.size()==0) return true;
    MapGrid2D::XYCell src = current_target;
    while (path.size()>0)
    {
        MapGrid2D::XYCell dst = path.front();
        path.pop();
        cvLine(map, cvPoint(src.x, src.y), cvPoint(dst.x, dst.y), color);
        src=dst;
    };
    return true;
}

bool map_utilites::drawCurrentPosition(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
{
    if (map==0) return false;
    cvCircle(map, cvPoint(current.x, current.y), 6, color);
    int orient_x = current.x + int(12 * cos(-angle));
    int orient_y = current.y + int(12 * sin(-angle));
    cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
    return true;
}

bool map_utilites::drawGoal(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
{
    if (map == 0) return false;
    cvCircle(map, cvPoint(current.x, current.y), 3, color);
    if (std::isnan(angle)==false)
    {
        int orient_x = current.x + int(6 * cos(-angle));
        int orient_y = current.y + int(6 * sin(-angle));
        cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
    }
    return true;
}

bool map_utilites::drawInfo(IplImage *map, MapGrid2D::XYCell current, MapGrid2D::XYCell orig, MapGrid2D::XYCell x_axis, MapGrid2D::XYCell y_axis, std::string status, const yarp::dev::Map2DLocation& localiz, const CvFont& font, const CvScalar& color)
{
    if (map==0) return false;
    char txt[255];
    if (1)
    {
        snprintf(txt, 255, "%s", status.c_str());
        cvPutText(map, txt, cvPoint(current.x, current.y - 15), &font, color);
    }
    if (1)
    {
        snprintf(txt, 255, "%.1f %.1f %.1f", localiz.x, localiz.y, localiz.theta);
        cvPutText(map, txt, cvPoint(current.x, current.y), &font, color);
    }
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(x_axis.x, x_axis.y), cvScalar(211, 0, 0));
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(y_axis.x, y_axis.y), cvScalar(0, 211, 0));
    return true;
}

bool map_utilites::drawLaserScan(IplImage *map, std::vector <MapGrid2D::XYCell>& laser_scan, const CvScalar& color)
{
    if (map==0) return false;
    for (unsigned int i=0; i<laser_scan.size(); i++)
    cvCircle(map, cvPoint(laser_scan[i].x, laser_scan[i].y), 0, color);
    return true;
}

bool map_utilites::drawLaserMap(IplImage *map, const yarp::dev::MapGrid2D& laserMap, const CvScalar& color)
{
    if (map==0) return false;
    for (size_t y=0; y<laserMap.height(); y++)
        for (size_t x=0; x<laserMap.width(); x++)
        {
            MapGrid2D::map_flags flag;
            laserMap.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag);
            if (flag==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE ||
                flag==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
            {
                cvCircle(map, cvPoint(x,y), 0, color);
            }
        }
    return true;
}

void map_utilites::update_obstacles_map(yarp::dev::MapGrid2D& map_to_be_updated, const yarp::dev::MapGrid2D& obstacles_map)
{
    //copies obstacles (and only them) from a source map to a destination map
    if (map_to_be_updated.width() != obstacles_map.width() ||
        map_to_be_updated.height() != map_to_be_updated.height())
    {
        yError() << "update_obstacles_map: the two maps must have the same size!";
        return;
    }
    for (size_t y=0; y<map_to_be_updated.height(); y++)
        for (size_t x=0; x<map_to_be_updated.width(); x++)
        {
            MapGrid2D::map_flags flag_src;
            MapGrid2D::map_flags flag_dst;
            map_to_be_updated.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag_dst);
            obstacles_map.getMapFlag(yarp::dev::MapGrid2D::XYCell (x,y), flag_src);
            if (flag_dst==MapGrid2D::MAP_CELL_FREE)
            {
                if      (flag_src==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE; }
                else if (flag_src==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE; }
            }
        }
}
