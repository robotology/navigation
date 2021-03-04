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

#define _USE_MATH_DEFINES
#include <cmath>

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

#include "map.h"

#ifndef DEG2RAD
#define DEG2RAD M_PI/180.0
#endif

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace map_utilites;

YARP_LOG_COMPONENT(NAVIGATION_GUI_MAP, "navigation.navigationGui.map")

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

bool map_utilites::drawPath(IplImage *map, XYCell current_position, XYCell current_target, std::queue<XYCell> path, const CvScalar& color1, const CvScalar& color2)
{
    if (map==0) return false;
    if (path.size()==0) return true;
    
    XYCell src = current_target;
    while (path.size()>0)
    {
        XYCell dst = path.front();
        path.pop();
        cvLine(map, cvPoint(src.x, src.y), cvPoint(dst.x, dst.y), color2);
        src=dst;
    };
    
    cvLine(map, cvPoint(current_position.x, current_position.y), cvPoint(current_target.x, current_target.y), color1);
    return true;
}

bool map_utilites::drawCurrentPosition(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info, const yarp::dev::Nav2D::Map2DLocation& current, const CvScalar& color)
{
    if (map == 0) return false;
    XYCell cell = info.toXYCell(current);
    double t = current.theta+info.m_origin.get_theta();

    cvCircle(map, cvPoint(cell.x, cell.y), 6, color);
    int orient_x = cell.x + int(12 * cos(-t * DEG2RAD));
    int orient_y = cell.y + int(12 * sin(-t * DEG2RAD));
    cvLine(map, cvPoint(cell.x, cell.y), cvPoint(orient_x, orient_y), color);
    return true;
}

bool map_utilites::drawGoal(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info, const yarp::dev::Nav2D::Map2DLocation current, const CvScalar& color)
{
    if (map == 0) return false;
    XYCell cell = info.toXYCell(current);
    double t = current.theta + info.m_origin.get_theta();

    cvCircle(map, cvPoint(cell.x, cell.y), 3, color);
    if (std::isnan(t) == false)
    {
        int orient_x = cell.x + int(6 * cos(-t * DEG2RAD));
        int orient_y = cell.y + int(6 * sin(-t * DEG2RAD));
        cvLine(map, cvPoint(cell.x, cell.y), cvPoint(orient_x, orient_y), color);
    }
    return true;
}

bool map_utilites::drawArea(IplImage *map, std::vector<XYCell> area, const CvScalar& color)
{
    if (map == 0) return false;
    if (area.size() < 3) return false;
    size_t last = area.size();
    for (size_t i = 0; i < last-1; i++)
    {
        cvLine(map, cvPoint(area[i].x, area[i].y), cvPoint(area[i+1].x, area[i+1].y), color);
    }
    cvLine(map, cvPoint(area[last-1].x, area[last-1].y), cvPoint(area[0].x, area[0].y), color);
    return true;
}

bool map_utilites::drawPose(IplImage* map, const yarp::dev::Nav2D::MapGrid2D& info, const yarp::dev::Nav2D::Map2DLocation current, const CvScalar& color)
{
    if (map == 0) return false;
    XYCell cell = info.toXYCell(current);
    double t = current.theta + info.m_origin.get_theta();

    cvCircle(map, cvPoint(cell.x, cell.y), 2, color);
    if (std::isnan(t) == false)
    {
        int orient_x = cell.x + int(6 * cos(-t * DEG2RAD));
        int orient_y = cell.y + int(6 * sin(-t * DEG2RAD));
        cvLine(map, cvPoint(cell.x, cell.y), cvPoint(orient_x, orient_y), color);
    }
    return true;
}

bool map_utilites::drawInfo(IplImage *map, XYCell current, XYCell orig, XYCell x_axis, XYCell y_axis, std::string status, const Map2DLocation& localiz, const CvFont& font, const CvScalar& color)
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

bool map_utilites::drawInfoFixed(IplImage *map, XYCell whereToDraw, XYCell orig, XYCell x_axis, XYCell y_axis, std::string status, const Map2DLocation& localiz, const CvFont& font, const CvScalar& color)
{
    if (map == 0) return false;
    char txt[255];
    if (1)
    {
        snprintf(txt, 255, "Loc= x:%.2f y:%.2f t:%.1f, status= %s", localiz.x, localiz.y, localiz.theta, status.c_str());
        cvPutText(map, txt, cvPoint(whereToDraw.x, whereToDraw.y), &font, color);
    }
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(x_axis.x, x_axis.y), cvScalar(211, 0, 0));
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(y_axis.x, y_axis.y), cvScalar(0, 211, 0));
    return true;
}

bool map_utilites::drawLaserScan(IplImage *map, std::vector <XYCell>& laser_scan, const CvScalar& color)
{
    if (map==0) return false;
    for (unsigned int i=0; i<laser_scan.size(); i++)
    cvCircle(map, cvPoint(laser_scan[i].x, laser_scan[i].y), 0, color);
    return true;
}

bool map_utilites::drawLaserMap(IplImage *map, const MapGrid2D& laserMap, const CvScalar& color)
{
    if (map==0) return false;
    for (size_t y=0; y<laserMap.height(); y++)
        for (size_t x=0; x<laserMap.width(); x++)
        {
            MapGrid2D::map_flags flag;
            laserMap.getMapFlag(XYCell (x,y), flag);
            if (flag==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE ||
                flag==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
            {
                cvCircle(map, cvPoint(x,y), 0, color);
            }
        }
    return true;
}

void map_utilites::update_obstacles_map(MapGrid2D& map_to_be_updated, const MapGrid2D& obstacles_map)
{
    //copies obstacles (and only them) from a source map to a destination map
    if (map_to_be_updated.width() != obstacles_map.width() ||
        map_to_be_updated.height() != map_to_be_updated.height())
    {
        yCError(NAVIGATION_GUI_MAP) << "update_obstacles_map: the two maps must have the same size!";
        return;
    }
    for (size_t y=0; y<map_to_be_updated.height(); y++)
        for (size_t x=0; x<map_to_be_updated.width(); x++)
        {
            MapGrid2D::map_flags flag_src;
            MapGrid2D::map_flags flag_dst;
            map_to_be_updated.getMapFlag(XYCell (x,y), flag_dst);
            obstacles_map.getMapFlag(XYCell (x,y), flag_src);
            if (flag_dst==MapGrid2D::MAP_CELL_FREE)
            {
                if      (flag_src==MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE; }
                else if (flag_src==MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE)
                { flag_dst=MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE; }
            }
        }
}
