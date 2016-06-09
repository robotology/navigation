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

struct cell
{
    int x;
    int y;
    cell()             {x=0; y=0;}
    cell(int u, int v) {x=u; y=v;}
};

class map_class
{
    public:
    int size_x;
    int size_y;
    int crop_x;
    int crop_y;
    int crop_w;
    int crop_h;
    double                                  resolution;
    yarp::sig::Vector                       origin;
    IplImage*                               loaded_map;
    IplImage*                               processed_map;
    IplImage*                               processed_map_with_scan;
    
    public:
    map_class();

    bool loadMap(string filename);
    bool crop(IplImage* img, IplImage* &imgOrig);
    bool enlargeObstacles(const IplImage* src, IplImage*& dst, unsigned int times);
    bool enlargeScan(std::vector <cell>& laser_scan, unsigned int times, double max_dist);
    bool skeletonize(const IplImage* src, IplImage*& dst);
    bool sendToPort (BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >* port, IplImage* image_to_send); 
    
    //return true if the straight line that connects src with dst does not contain any obstacles
    bool checkStraightLine(IplImage* map, cell src, cell dst);

    //simplify the path
    bool simplifyPath(IplImage *map, std::queue<cell> input_path, std::queue<cell>& output_path);

    //draw stuff on the map
    void drawPath(IplImage *map, cell current_position, cell current_target, std::queue<cell> path, const CvScalar& color);
    void drawCurrentPosition(IplImage *map, cell current, const CvScalar& color);
    void drawLaserScan(IplImage *map, std::vector <cell>& laser_scan, const CvScalar& color);

    //compute the path
    bool findPath(IplImage *img, cell start, cell goal, std::queue<cell>& path);

    cell world2cell (yarp::sig::Vector v); 
    yarp::sig::Vector cell2world (cell c);

    private:
    
    //used by enlargemetn functions
    void enlargePixelObstacle (int& i, int& j, cv::Mat& src, cv::Mat& dst, cv::Vec3b& color, std::vector<cv::Vec2d>& next_list);
    void enlargePixelSkeleton (int& i, int& j, cv::Mat& src, cv::Mat& dst, cv::Vec3b& color, std::vector<cv::Vec2d>& next_list);
};

#endif
