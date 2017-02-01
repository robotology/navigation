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
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/IAnalogSensor.h>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h> 

#include "map.h"
#include "aStar.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

bool sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send)
{
    if (port!=0 && port->getOutputCount()>0)
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *segImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
        segImg->resize(image_to_send->width, image_to_send->height );
        cvCopyImage(image_to_send, (IplImage*)segImg->getIplImage());
        port->prepare() = *segImg;
        port->write();
        delete segImg;
        return true;
    }
    return false;
}

bool enlargeScan(std::vector <MapGrid2D::XYCell>& laser_scan, unsigned int times, double max_dist)
{   
    //@@@ THIS FUNCTION IS STILL TO BE COMPLETED
    std::vector <MapGrid2D::XYCell> laser_scan2;
    if (max_dist>0)
    {
        for (unsigned int i=0; i<laser_scan.size(); i++)
        {
            int dx = (laser_scan[i].x-laser_scan[i].x);
            int dy = (laser_scan[i].x-laser_scan[i].x);
            if (dx*dx + dy*dy < max_dist)
                laser_scan2.push_back(laser_scan[i]);
        }
    }

    for (unsigned int i=0; i< times; i++)
    {

    }
    return true;
}

bool simplifyPath(yarp::dev::MapGrid2D& map, std::queue<MapGrid2D::XYCell> input_path, std::queue<MapGrid2D::XYCell>& output_path)
{
    unsigned int path_size = input_path.size();
    if (path_size==0) return false;

    output_path.push(input_path.front());
    
    //make a copy of the path in a vector
    std::vector <MapGrid2D::XYCell> path;
    for (unsigned int i=0; i<path_size; i++)
    {
        MapGrid2D::XYCell tmp = input_path.front();
        input_path.pop();
        path.push_back(tmp);
    }

    for (unsigned int i=0; i<path_size; i++)
    {
        MapGrid2D::XYCell start_cell = path.at(i);
        MapGrid2D::XYCell old_stop_cell = start_cell;
        MapGrid2D::XYCell best_old_stop_cell = start_cell;
        MapGrid2D::XYCell stop_cell = start_cell;
        MapGrid2D::XYCell best_stop_cell = start_cell;
        unsigned int j=i+1;
        unsigned int best_j=j;
        for (; j<path_size; j++)
        {
            old_stop_cell = path.at(j-1);
            stop_cell     = path.at(j);
            //yDebug ("%d %d -> %d %d\n", start_cell.x, start_cell.y, stop_cell.x, stop_cell.y);
            if (checkStraightLine(map, start_cell, stop_cell))
            {
                best_old_stop_cell=old_stop_cell;
                best_stop_cell=stop_cell;
                best_j = j;
            }
        };
        if (best_j==path_size)
        {
            output_path.push(best_stop_cell);
            return true;
        }
        else
        {
            output_path.push(best_old_stop_cell);
            i=best_j-1;
        }
    };
    return true;
};

void drawPath(IplImage *map, MapGrid2D::XYCell current_position, MapGrid2D::XYCell current_target, std::queue<MapGrid2D::XYCell> path, const CvScalar& color)
{
    if (map==0) return;
    cvLine(map, cvPoint(current_position.x, current_position.y), cvPoint(current_target.x, current_target.y), color);

    if (path.size()==0) return;
    MapGrid2D::XYCell src = current_target;
    while (path.size()>0)
    {
        MapGrid2D::XYCell dst = path.front();
        path.pop();
        cvLine(map, cvPoint(src.x, src.y), cvPoint(dst.x, dst.y), color);
        src=dst;
    };
}

void drawCurrentPosition(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
{
    if (map==0) return;
    cvCircle(map, cvPoint(current.x, current.y), 6, color);
    int orient_x = current.x + 12 * cos(-angle);
    int orient_y = current.y + 12 * sin(-angle);
    cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
}

void drawGoal(IplImage *map, MapGrid2D::XYCell current, double angle, const CvScalar& color)
{
    if (map == 0) return;
    cvCircle(map, cvPoint(current.x, current.y), 3, color);
    if (isnan(angle)==false)
    {
        int orient_x = current.x + 6 * cos(-angle);
        int orient_y = current.y + 6 * sin(-angle);
        cvLine(map, cvPoint(current.x, current.y), cvPoint(orient_x, orient_y), color);
    }
}

void drawInfo(IplImage *map, MapGrid2D::XYCell current, MapGrid2D::XYCell orig, MapGrid2D::XYCell x_axis, MapGrid2D::XYCell y_axis, const yarp::dev::Map2DLocation& localiz, const CvFont& font, const CvScalar& color)
{
    char txt[255];
    sprintf(txt, "%.1f %.1f %.1f", localiz.x, localiz.y, localiz.theta);
    cvPutText(map, txt, cvPoint(current.x, current.y), &font, color);
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(x_axis.x, x_axis.y), cvScalar(211, 0, 0));
    cvLine(map, cvPoint(orig.x, orig.y), cvPoint(y_axis.x, y_axis.y), cvScalar(0, 211, 0));
}

void drawLaserScan(IplImage *map, std::vector <MapGrid2D::XYCell>& laser_scan, const CvScalar& color)
{
    if (map==0) return;
    for (unsigned int i=0; i<laser_scan.size(); i++)
    cvCircle(map, cvPoint(laser_scan[i].x, laser_scan[i].y), 0, color);
}

bool checkStraightLine(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell src, MapGrid2D::XYCell dst)
{
    //here using the fast Bresenham algorithm
    int dx = abs(dst.x-src.x);
    int dy = abs(dst.y-src.y); 
    int err = dx-dy;
    
    int sx;
    int sy;
    if (src.x < dst.x) sx = 1; else sx = -1;
    if (src.y < dst.y) sy = 1; else sy = -1;
    
    while(1)
    {
        if (map.isFree(src) == false) return false;
        if (src.x==dst.x && src.y==dst.y) break;
        int e2 = err*2;
        if (e2 > -dy)
        {
            err = err-dy;
            src.x += sx;
        }
        if (e2 < dx)
        {
            err = err+dx;
            src.y += sy;
        }
    }
    return true;
}

bool findPath(yarp::dev::MapGrid2D& map, MapGrid2D::XYCell start, MapGrid2D::XYCell goal, std::queue<MapGrid2D::XYCell>& path)
{
    //return find_dijkstra_path(map, start, goal, path);
    return find_astar_path(map, start, goal, path);
}
/*
bool map_class::crop(IplImage *img, IplImage* &dest)
{
    int top = -1;
    int left = -1;
    int right = -1;
    int bottom = -1;

    cv::Mat imgMat = img;    
 
    for (int j=0;j<imgMat.rows;j++){
        for (int i=0;i<imgMat.cols;i++){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    top = j; 
                    goto topFound;
            }
        }
    }

    topFound:
    for (int j=imgMat.rows-1; j>0; j--){
        for (int i=imgMat.cols-1; i>0 ;i--){
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    bottom = j+1; 
                    goto bottomFound;
            }
        }
    }

    bottomFound:
    for (int i=0;i<imgMat.cols;i++){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    left = i; 
                    goto leftFound;
            }        
       }
    }
    
    leftFound:
    for (int i=imgMat.cols-1;i>0;i--){
        for (int j=0;j<imgMat.rows;j++){    
            if ( imgMat.at<cv::Vec3b>(j,i)[0] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[1] != 205 &&
                 imgMat.at<cv::Vec3b>(j,i)[2] != 205 )
            {
                    right = i; 
                    goto rightFound;
            }        
       }
    }
    
    rightFound:

    cvSetImageROI (img, cvRect(left, top, right-left, bottom-top) );   
    if (dest != 0)
        cvReleaseImage (&dest);
    dest = cvCreateImage(cvSize(right-left,  bottom-top), IPL_DEPTH_8U, 3 );
    cvCopyImage(img, dest); 

    m_crop_x = left;
    m_crop_y = top;
    m_crop_w = right;
    m_crop_h = bottom;

    return true;
}
*/
/*
cell map_class::world2Cell (yarp::sig::Vector v)
{
    cell c;
    c.x = int((v[0]-this->origin[0])/this->resolution);
    c.y = int((-v[1]-this->origin[1])/this->resolution);
    c.x -= this->crop_x;
    c.y -= this->crop_y;
    return c;
}

yarp::sig::Vector map_class::cell2World (cell c)
{
    c.x += this->crop_x;
    c.y += this->crop_y;
    yarp::sig::Vector v(2);
    v[0] = double(c.x)*this->resolution;
    v[1] = double(c.y)*this->resolution;
    v[0] = v[0]+this->origin[0];
    v[1] = -(v[1]+this->origin[1]);
    return v;
}
*/