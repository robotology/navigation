#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cv.h>
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


//https://docs.opencv.org/3.1.0/d2/dbd/tutorial_distance_transform.html

static bool getImage(cv::Mat &img, std::string mapname);

int main( int argc, char** argv)
{

    cv::Mat img;
    std::string mapname="";

    if(argc>=2)
        mapname=argv[1];
    else
        mapname="testMap";
    if(!getImage(img, mapname))
    {
        std::cout << "Error in get image" << std::endl;
        return 0;
    }

//     if(img.rows>1000)
//     {
//         cv::resize(img, img, cv::Size(), 0.4, 0.4);
//     }


    std::cout << "Num of white pixel is " << countNonZero(img) <<std::endl;

    //show the faces
    namedWindow( "imageReal", 1 );
    imshow( "imageReal", img );

    // Perform the distance transform algorithm
    cv::Mat dist;
    distanceTransform(img, dist, CV_DIST_L2, 3);
    // Normalize the distance image for range = {0.0, 1.0)
    normalize(dist, dist, 0, 1., cv::NORM_MINMAX);

    namedWindow( "distNorm", 1 );
    imshow( "distNorm", dist );

    // Threshold to obtain the peaks
    threshold(dist, dist, .4, 1., CV_THRESH_BINARY);


    namedWindow( "distThresh", 1 );
    imshow( "distThresh", dist );

    // Dilate a bit the dist image
    cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
    dilate(dist, dist, kernel1);

    namedWindow( "distDilate", 1 );
    imshow( "distDilate", dist );

    waitKey();

    // Create the CV_8U version to run the contour finder
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find the contours
    std::vector<std::vector<cv::Point> > cnt;

    findContours(dist_8u,cnt,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);


    // Get the moments and mass center (future use)
    std::vector<cv::Moments> mu( cnt.size() );
    std::vector<cv::Point2f> mc( cnt.size() );
    // Collect all the seed points (future use)
    std::vector<cv::Point> seeds;

    //create marker image for watershed
    cv::Mat markers = cv::Mat::zeros(img.size(), CV_32SC1);

    for( size_t i = 0; i< cnt.size(); i++ )
    {
        // Draw the forground marker
        drawContours(markers, cnt, static_cast<int>(i), cv::Scalar::all(static_cast<int>(i)+1), CV_FILLED);

        //collect moments and mass center (mc) of each contours. Currently I don't need them, maybe later....
        mu[i] = moments( cnt[i], false );
        mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        seeds.push_back(mc[i]);
    }

    // Draw the background marker because watershed algorithm needs it
    circle(markers, cv::Point(5,5), 3, cv::Scalar(255), -1);

    namedWindow( "Marker1000", 1 );
    imshow( "Marker1000", markers*10000 );


    cv::cvtColor(img, img, CV_GRAY2RGB);

    watershed(img, markers);

    namedWindow( "watershed", 1 );
    imshow( "watershed", markers*10000 );

//     Mat im_gray;
//
//     markers.convertTo(im_gray, CV_8U);
//     bitwise_not(im_gray, im_gray);
//    // Mat img_bw = im_gray > 128;
//
//     namedWindow( "img_bw", 1 );
//     imshow( "img_bw", im_gray );



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

    namedWindow( "out", 1 );
    imshow( "out", out_cv );

    Mat im_gray;
    cvtColor(out_cv, im_gray, COLOR_RGB2GRAY);

    namedWindow( "img_gray", 1 );
    imshow( "img_gray", im_gray );

    erode(im_gray, im_gray, kernel1);
    namedWindow( "erode", CV_WINDOW_AUTOSIZE );
    imshow( "erode", im_gray );


    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;
    //findContours( im_gray, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    findContours( im_gray, contours0, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    Mat drawing0= Mat::zeros( im_gray.size(), CV_8UC1 );
    Mat drawing1= Mat::zeros( im_gray.size(), CV_8UC1 );
    Mat drawing2= Mat::zeros( im_gray.size(), CV_8UC1 );

    vector<Mat> drawingList(contours0.size());
    vector<double> minArea (contours0.size());
    vector<double> maxArea (contours0.size());

    //std::cout<< "contours0.size()= "<<contours0.size()<< " hierarchy.size()="<<hierarchy.size() <<std::endl;
    std::cout << "Number of found rooms is " << contours0.size() << std::endl;
    for(int i=0; i<contours0.size(); i++)
    {
//         std::cout << "hierarchy["<<i<<"][0]=" << hierarchy[i][0] << "  ";
//         std::cout << "hierarchy["<<i<<"][1]=" << hierarchy[i][1] << "  ";
//         std::cout << "hierarchy["<<i<<"][2]=" << hierarchy[i][2] << "  ";
//         std::cout << "hierarchy["<<i<<"][3]=" << hierarchy[i][3] << "  ";
//         std::cout << std::endl;
        //calculate area after erode
        minArea[i]=contourArea(contours0[i]);

        //init th i-th room and drow the i-th room
        drawingList[i]=drawing0.clone();
        drawContours(drawingList[i], contours0, i, cv::Scalar(255), 1);

        //dilate in order to restore the initial size or little bigger
        dilate(drawingList[i], drawingList[i], kernel1);
        vector<vector<Point> > contours;
        findContours( drawingList[i], contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

        //calculate the max area
        maxArea[i]=contourArea(contours[0]);
//         std::cout << "drawing num" << i <<"  has area " << contourArea(contours[0]) << "  contours.size=" << contours.size() << std::endl;
//         std::cout << "Disconnected region num=" << i << " has area= " << contourArea(contours0[i]) << std::endl;

        std::cout << "the room number " << i << " has min-area=" << minArea[i] << " and max-area=" << maxArea[i] << std::endl;

        std::string namewin="cont";
        namedWindow(namewin+to_string(i), CV_WINDOW_AUTOSIZE);
        imshow( namewin+to_string(i), drawingList[i] );


    }

//     drawContours( drawing0, contours0, 0, cv::Scalar(255), 1 );
//     namedWindow( "cont0", CV_WINDOW_AUTOSIZE );
//     imshow( "cont0", drawingList[0] );
//
//     drawContours( drawing1, contours0, 1, cv::Scalar(255), 1 );
//     namedWindow( "cont1", CV_WINDOW_AUTOSIZE );
//     imshow( "cont1", drawingList[1] );
//
//
//     drawContours( drawing2, contours0, 2, cv::Scalar(255), 1 );
//     namedWindow( "con2", CV_WINDOW_AUTOSIZE );
//     imshow( "cont2", drawingList[2] );
//
//
//     dilate(drawing0, drawing0, kernel1);
//     findContours( drawing0, contours0, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//     std::cout << "drawing0  has area " << contourArea(contours0[0]) << "  contours.size=" << contours0.size() << std::endl;
//
//     dilate(drawing1, drawing1, kernel1);
//     findContours( drawing1, contours0, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//     std::cout << "drawing1  has area " << contourArea(contours0[0]) << "  contours.size=" << contours0.size() << std::endl;
//
//     dilate(drawing2, drawing2, kernel1);
//     findContours( drawing2, contours0, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//     std::cout << "drawing1  has area " << contourArea(contours0[0]) << "  contours.size=" << contours0.size() << std::endl;


//     // Threshold to obtain the peaks
//     threshold(im_gray, im_gray, 1, 255, CV_THRESH_BINARY);
//     namedWindow( "img_th", 1 );
//     imshow( "img_th", im_gray );
//
//     bitwise_not(im_gray, im_gray);
//
//     namedWindow( "img_bw", 1 );
//     imshow( "img_bw", im_gray );







//     Mat canny_output, im_gray;
//     vector<vector<Point> > contours;
//     vector<Vec4i> hierarchy;
//     int thresh=100;
//     RNG rng(155);
//     cvtColor(out_cv, im_gray, COLOR_RGB2GRAY);
//     Canny( out_cv, canny_output, thresh, thresh*2, 3 );
//     namedWindow( "canny", CV_WINDOW_AUTOSIZE );
//     imshow( "canny", canny_output );
//
//  //   cv::Mat kernel1 = cv::Mat::ones(3, 3, CV_8UC1);
//
//     erode(canny_output, canny_output, kernel1);
//     namedWindow( "canny+erode", CV_WINDOW_AUTOSIZE );
//     imshow( "canny+erode", canny_output );
//
//     dilate(canny_output, canny_output, kernel1);
//     namedWindow( "canny+erode+dilate", CV_WINDOW_AUTOSIZE );
//     imshow( "canny+erode+dilate", canny_output );
//
//     /// Find contours
//     findContours( canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );
//
//     /// Draw contours
//     Mat drawing = Mat::zeros( canny_output.size(), CV_8UC1 );
//     for( int i = 0; i< contours.size(); i++ )
//     {
//         std::cout << "hierarchy["<<i<<"][0]=" << hierarchy[i][0] << "  ";
//         std::cout << "hierarchy["<<i<<"][1]=" << hierarchy[i][1] << "  ";
//         std::cout << "hierarchy["<<i<<"][2]=" << hierarchy[i][2] << "  ";
//         std::cout << "hierarchy["<<i<<"][3]=" << hierarchy[i][3] << "  ";
//         std::cout << std::endl;
//         std::cout << "Disconnected region num=" << i << " has area= " << contourArea(contours[i]) << std::endl;
//         drawContours( drawing, contours, i, cv::Scalar::all(static_cast<int>(i)+1), 2 );
//     }
//
//     namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
//     imshow( "Contours", drawing*10000 );

//     vector<vector<Point> > contours0;
//     findContours( drawing, contours0, hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//
//     std::cout<< "contours0.size()= "<<contours0.size()<< " hierarchy.size()="<<hierarchy.size() <<std::endl;
//     for(int i=0; i<contours0.size(); i++)
//     {
//         std::cout << "hierarchy["<<i<<"][0]=" << hierarchy[i][0] << "  ";
//         std::cout << "hierarchy["<<i<<"][1]=" << hierarchy[i][1] << "  ";
//         std::cout << "hierarchy["<<i<<"][2]=" << hierarchy[i][2] << "  ";
//         std::cout << "hierarchy["<<i<<"][3]=" << hierarchy[i][3] << "  ";
//         std::cout << std::endl;
//
//         std::cout << "Disconnected region num=" << i << " has area= " << contourArea(contours0[i]) << std::endl;
//
//     }


//         Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//     cvtColor(out_cv, im_gray, COLOR_BGR2GRAY);
//     // Threshold to obtain the peaks
//     threshold(im_gray, im_gray, 1, 255, CV_THRESH_BINARY);
//     namedWindow( "img_th", 1 );
//     imshow( "img_th", im_gray );
//
//    bitwise_not(im_gray, im_gray);
//
//     namedWindow( "img_bw", 1 );
//    imshow( "img_bw", im_gray );
//
//
//     vector<vector<Point> > contours0;
//     vector<Vec4i> hierarchy;
//     findContours( im_gray, contours0, hierarchy, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//
//     std::cout<< "contours0.size()= "<<contours0.size()<< " hierarchy.size()="<<hierarchy.size() <<std::endl;
//     for(int i=0; i<contours0.size(); i++)
//     {
//         std::cout << "hierarchy["<<i<<"][0]=" << hierarchy[i][0] << "  ";
//         std::cout << "hierarchy["<<i<<"][1]=" << hierarchy[i][1] << "  ";
//         std::cout << "hierarchy["<<i<<"][2]=" << hierarchy[i][2] << "  ";
//         std::cout << "hierarchy["<<i<<"][3]=" << hierarchy[i][3] << "  ";
//         std::cout << std::endl;
//
//         std::cout << "Disconnected region num=" << i << " has area= " << contourArea(contours0[i]) << std::endl;
//
//     }




    waitKey();

    return 0;
}


#define FROM_MAPS 1
static bool getImage(cv::Mat &img, std::string mapname)
{
#ifdef FROM_MAPS
    IMap2D*              m_iMap = nullptr;
    PolyDriver           m_pMap;

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
    MapGrid2D the_map;
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
    img = Mat::zeros(w,h, CV_8UC1);
    for (auto x=0; x<w; x++)
        for (auto y = 0; y <h; y++)
        {
            unsigned char c = 0;
            yarp::dev::MapGrid2D::map_flags pixin;
            the_map.getMapFlag(yarp::dev::MapGrid2D::XYCell(x, y), pixin);
            if      (pixin == yarp::dev::MapGrid2D::MAP_CELL_WALL) { c = 0;}
            else if (pixin == yarp::dev::MapGrid2D::MAP_CELL_UNKNOWN) { c = 0; }
            else if (pixin == yarp::dev::MapGrid2D::MAP_CELL_FREE) { c = 255; }
            else if (pixin == yarp::dev::MapGrid2D::MAP_CELL_KEEP_OUT) { c = 255; }
            else if (pixin == yarp::dev::MapGrid2D::MAP_CELL_ENLARGED_OBSTACLE) { c = 255;   }
            else if (pixin == yarp::dev::MapGrid2D::MAP_CELL_TEMPORARY_OBSTACLE) { c = 255;  }
            img.at<unsigned char>(x, y) = c;
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
