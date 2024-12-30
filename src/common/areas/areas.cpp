/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Map2DArea.h>
#include <yarp/dev/iMap2D.h>
#include <yarp/math/Vec2D.h>
#include <yarp/dev/Polydriver.h>
#include <yarp/os/Logstream.h>

using namespace cv;
using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::dev::Nav2D;

const int w = 500;
int levels = 3;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
static void on_trackbar(int, void*)
{
    Mat cnt_img = Mat::zeros(w, w, CV_8UC3);
    int _levels = levels - 3;
    for (auto contour_id = 0; contour_id < contours.size(); contour_id++)
    {
        drawContours(cnt_img, contours, contour_id, Scalar(128, 255, 255), 2, LINE_AA, hierarchy, std::abs(_levels));
    }
    imshow("contours", cnt_img);
}

//https://answers.opencv.org/question/59782/how-to-detect-contours-for-two-or-more-than-two-specific-colors/

int main( int argc, char** argv)
{
#ifdef SIMULATION_ONLY
    Mat img = Mat::zeros(w, w, CV_8UC1);
    for (int i = 0; i < 1; i++)
    {
        int dx = (i % 2) * 250 - 30;
        int dy = (i / 2) * 150;
        const Scalar white = Scalar(255);
        const Scalar black = Scalar(0);

        rectangle(img, Point(50, 50), Point(100, 100), white, -1);
        rectangle(img, Point(10, 70), Point(210, 90), white, -1);
        rectangle(img, Point(150, 50), Point(200, 100), white, -1);
    }
#else
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
    std::string map_name = "testMap";
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
    Mat img = Mat::zeros(w,h, CV_8UC1);
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
            img.at<unsigned char>(x, y) = c;
        }
#endif

    namedWindow( "image", 1 );
    imshow( "image", img );
    //Extract the contours so that
    vector<vector<Point> > contours0;
    findContours( img, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    contours.resize(contours0.size());
    for (size_t k = 0; k < contours0.size(); k++)
    {
        Map2DArea area;
        double accuracy = 4;
        approxPolyDP(Mat(contours0[k]), contours[k], accuracy, true);
#if SIMULATION_ONLY
        area.map_id = "test";
#else
        area.map_id = the_map.getMapName();
        double resolution = 1.0; the_map.getResolution(resolution);
#endif
        for (size_t kk = 0; kk < contours[k].size(); kk++)
        {
#if SIMULATION_ONLY
            area.points.push_back(yarp::math::Vec2D<double>(contours[k][kk].x, contours[k][kk].y));
            cout << k << " " << kk << " x:" << contours[k][kk].x << " y:" << contours[k][kk].y << endl;
#else
            double xx = contours[k][kk].x*resolution;
            double yy = contours[k][kk].y*resolution;
            area.points.push_back(yarp::math::Vec2D<double>(xx, yy));
            cout << k << " " << kk << " x:" << contours[k][kk].x << "(" << xx << ") y:" << contours[k][kk].y << " (" << yy << ")"<<endl;
#endif
        }
    }

    namedWindow( "contours", 1 );
    createTrackbar( "levels+3", "contours", &levels, 7, on_trackbar );
    on_trackbar(0,0);
    waitKey();
    return 0;
}