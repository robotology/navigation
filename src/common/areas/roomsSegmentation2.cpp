#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Map2DArea.h>
#include <yarp/math/Vec2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

#define FROM_MAPS 1
//#define DRAW_WINDOWS 1

IMap2D*              m_iMap = nullptr;
PolyDriver           m_pMap;
std::string          mapname = "";
MapGrid2D            the_map;

///https://docs.opencv.org/3.1.0/d2/dbd/tutorial_distance_transform.html

static bool getImage(cv::Mat &img, std::string mapname);


int main( int argc, char** argv)
{

    cv::Mat img;


    if(argc>=2)
        mapname=argv[1];
    else
        mapname="testMap2";
    if(!getImage(img, mapname))
    {
        yDebug() << "Error in get image";
        return 0;
    }

//     if(img.rows>1000)
//     {
//         cv::resize(img, img, cv::Size(), 0.4, 0.4);
//     }


    yDebug() << "Num of white pixel is " << countNonZero(img);
    //show the faces
#ifdef DRAW_WINDOWS
    namedWindow( "imageReal", 1 );
    imshow( "imageReal", img );
#endif

    // Perform the distance transform algorithm
    cv::Mat dist;
    distanceTransform(img, dist, DistanceTypes::DIST_L2, 3);
    // Normalize the distance image for range = {0.0, 1.0)
    normalize(dist, dist, 0, 1., cv::NORM_MINMAX);

#ifdef DRAW_WINDOWS
    namedWindow( "distNorm", 1 );
    imshow( "distNorm", dist );
#endif
    // Threshold to obtain the peaks
    threshold(dist, dist, .4, 1., ThresholdTypes::THRESH_BINARY);

#ifdef DRAW_WINDOWS
    namedWindow( "distThresh", 1 );
    imshow( "distThresh", dist );
#endif

    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    dilate(dist, dist, kernel1);

#ifdef DRAW_WINDOWS
    namedWindow( "distDilate", 1 );
    imshow( "distDilate", dist );
#endif

    //waitKey();

    // Create the CV_8U version to run the contour finder
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find the contours
    std::vector<std::vector<cv::Point> > cnt;

    findContours(dist_8u,cnt, RetrievalModes::RETR_EXTERNAL, ContourApproximationModes::CHAIN_APPROX_NONE);


    // Get the moments and mass center (future use)
    std::vector<cv::Moments> mu( cnt.size() );
    std::vector<cv::Point2f> mc( cnt.size() );
    // Collect all the seed points (future use)
    std::vector<cv::Point> seeds;

    //create marker image for watershed
    cv::Mat markers = cv::Mat::zeros(img.size(), CV_32SC1);

    for( size_t i = 0; i< cnt.size(); i++ )
    {
        // Draw the foreground marker
        drawContours(markers, cnt, static_cast<int>(i), cv::Scalar::all(static_cast<int>(i)+1), LineTypes::FILLED);

        //collect moments and mass center (mc) of each contours. Currently I don't need them, maybe later....
        mu[i] = moments( cnt[i], false );
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        seeds.push_back(mc[i]);
    }

    // Draw the background marker because watershed algorithm needs it
    circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);

#ifdef DRAW_WINDOWS
    namedWindow( "Marker1000", 1 );
    imshow( "Marker1000", markers*10000 );
#endif

    cv::cvtColor(img, img, ColorConversionCodes::COLOR_GRAY2RGB);

    watershed(img, markers);

#ifdef DRAW_WINDOWS
    namedWindow( "watershed", 1 );
    imshow( "watershed", markers*10000 );
#endif

    // Generate random colors
    std::vector<cv::Vec3b> colors;
    for (size_t i = 0; i < seeds.size(); i++)
    {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);
        colors.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
    }

    cv::Mat out_cv = cv::Mat::zeros(img.size(), CV_8UC3);

    // Fill labeled objects with random colors
    for (int i = 0; i < markers.rows; i++)
    {
        for (int j = 0; j < markers.cols; j++)
        {
            int index = markers.at<int>(i,j);
            if (index > 0 && index <= static_cast<int>(cnt.size()))
                out_cv.at<cv::Vec3b>(i,j) = colors[index-1];
            else
                out_cv.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
        }
    }

#ifdef DRAW_WINDOWS
    namedWindow( "out", 1 );
    imshow( "out", out_cv ); //color output image
#endif

    Mat im_gray;
    cvtColor(out_cv, im_gray, COLOR_RGB2GRAY);

#ifdef DRAW_WINDOWS
    namedWindow( "img_gray", 1 );
    imshow( "img_gray", im_gray );//grayscale output image
#endif

    erode(im_gray, im_gray, kernel1);

#ifdef DRAW_WINDOWS
    namedWindow( "erode", CV_WINDOW_AUTOSIZE );//erode on grayscale 
    imshow( "erode", im_gray );
#endif

    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;
    //findContours( im_gray, contours0, hierarchy, RETR_TREE, ContourApproximationModes::CHAIN_APPROX_SIMPLE);
    findContours( im_gray, contours0, RETR_EXTERNAL, ContourApproximationModes::CHAIN_APPROX_NONE);

    Mat drawing0= Mat::zeros( im_gray.size(), CV_8UC1 );

    vector<Mat> drawingList(contours0.size()); //vector of rooms 
    vector<double> minArea (contours0.size());
    vector<double> maxArea (contours0.size());

    yDebug() << "Number of found rooms is " << contours0.size();
    for(int i=0; i<contours0.size(); i++)//for each room
    {
        //calculate area after erode
        minArea[i]=contourArea(contours0[i]);

        //init th i-th room and draw the i-th room
        drawingList[i]=drawing0.clone();
        drawContours(drawingList[i], contours0, i, cv::Scalar(255), 1); //i-th room equivalent to contours0[0]

        //dilate in order to restore the initial size or little bigger
        dilate(drawingList[i], drawingList[i], kernel1);
        vector<vector<Point> > contours;

        //find contours, only external pixels?
        findContours( drawingList[i], contours, RETR_EXTERNAL, ContourApproximationModes::CHAIN_APPROX_NONE);

        //calculate the max area
        maxArea[i]=contourArea(contours[0]); //one only contour for i-th room
//         yDebug() << "drawing num" << i <<"  has area " << contourArea(contours[0]) << "  contours.size=" << contours.size();
//         yDebug() << "Disconnected region num=" << i << " has area= " << contourArea(contours0[i]);

        yDebug() << "the room number " << i << " has min-area=" << minArea[i] << " and max-area=" << maxArea[i];

#ifdef DRAW_WINDOWS
        std::string namewin="cont";
        namedWindow(namewin+to_string(i), CV_WINDOW_AUTOSIZE);
        imshow( namewin+to_string(i), drawingList[i] );
#endif

        //Extract the contours so that
        vector<vector<Point> > contours11;

        //contours11.resize(contours[0].size());
        contours11.resize(1);

        Map2DArea area;
        double accuracy = 3;
        approxPolyDP(Mat(contours0[i]), contours11[0], accuracy, true);
#if SIMULATION_ONLY
        area.map_id = "test";
#else
        area.map_id = mapname;
        double resolution = 0.05; //the_map.getResolution(resolution);
#endif
        for (size_t kk = 0; kk < contours11[0].size(); kk++)
        {
#if SIMULATION_ONLY
            area.points.push_back(yarp::math::Vec2D<double>(contours[k][kk].x, contours[k][kk].y));
            yDebug() << k << " " << kk << " x:" << contours[k][kk].x << " y:" << contours[k][kk].y;
#else
            XYWorld worldc = the_map.cell2World(XYCell(contours11[0][kk].x, contours11[0][kk].y));
            area.points.push_back(yarp::math::Vec2D<double>(worldc.x, worldc.y));
            yDebug() << i << " " << kk << " x:" << contours11[0][kk].x << "(" << worldc.x << ") y:" << contours11[0][kk].y << " (" << worldc.y << ")";
#endif
        }

        yDebug() << "***";
        drawingList[i] = drawing0.clone();
        drawContours(drawingList[i], contours11, 0, cv::Scalar(255), 1);

#ifdef FROM_MAPS
        std::string area_name = "auto_area" + to_string(i);
        if (m_iMap->storeArea(area_name, area))
        {
            yInfo() << "Area " << area_name << "successfully stored into map server";
        }
        else
        {
            yError() << "Area " << area_name << "failed to store into map server";
        }
#endif

#ifdef DRAW_WINDOWS
        namedWindow("final"+ to_string(i), CV_WINDOW_AUTOSIZE);
        imshow("final" + to_string(i), drawingList[i]);
#endif
    }

    //wait key
    waitKey();

    return 0;
}


static bool getImage(cv::Mat &img, std::string mapname)
{
#ifdef FROM_MAPS
    //open the map interface
    yarp::os::Property map_options;
    map_options.put("device", "map2DClient");
    map_options.put("local", std::string("/area") + "/map2DClient");
    map_options.put("remote", "/mapServer");
    if (m_pMap.open(map_options) == false)
    {
        yError() << "Unable to open map2DClient";
        return false;
    }
    m_pMap.view(m_iMap);
    if (m_iMap == 0)
    {
        yError() << "Unable to open map interface";
        return false;
    }

    //get the map
    std::string map_name = mapname;
    bool b = m_iMap->get_map(map_name, the_map);
    if (b == false)
    {
        yError() << "Map not found";
        return false;
    }

    yarp::sig::ImageOf<yarp::sig::PixelMono> the_grid;
    bool b2= the_map.getOccupancyGrid(the_grid);

    size_t w = the_map.width();
    size_t h = the_map.height();
    yInfo() << "get image of size " << w << h;
    img = Mat::zeros((int)h, (int)w, CV_8UC1);
    for (auto x=0; x<w; x++)
        for (auto y = 0; y <h; y++)
        {
            unsigned char c = 0;
            MapGrid2D::map_flags pixin;
            the_map.getMapFlag(XYCell(x, y), pixin);
            if      (pixin == MapGrid2D::MAP_CELL_WALL) { c = 0;}
            else if (pixin == MapGrid2D::MAP_CELL_UNKNOWN) { c = 0; }
            else if (pixin == MapGrid2D::MAP_CELL_FREE) { c = 255; }
            else if (pixin == MapGrid2D::MAP_CELL_KEEP_OUT) { c = 255; }
            else if (pixin == MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE) { c = 255;   }
            else if (pixin == MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE) { c = 255;  }
            img.at<unsigned char>(y, x) = c;
        }

    return true;
#else
    img = cv::Mat::zeros(500, 500, CV_8UC1);

    // Create first room
    rectangle(img, cv::Point(50, 50),  cv::Point(100, 100), cv::Scalar{255,255,255}, -1);
    // Create  second room
    rectangle(img, cv::Point(150, 20), cv::Point(200, 100), cv::Scalar{255,255,255}, -1);
    // Create link between fist room to corridor
    rectangle(img, cv::Point(90, 100), cv::Point(95, 110), cv::Scalar{255,255,255}, -1);
    // Create link between second room to corridor
    rectangle(img, cv::Point(190, 100), cv::Point(195, 110), cv::Scalar{255,255,255}, -1);
    // Create long corridor
    rectangle(img, cv::Point(10, 110),  cv::Point(210, 130), cv::Scalar{255,255,255}, -1);
    return true;
#endif
}
