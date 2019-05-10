#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <cv.h>
#include <math.h>
#include <iostream>
#include <yarp/dev/MapGrid2D.h>
#include <yarp/dev/Map2DArea.h>
#include <yarp/math/Vec2D.h>

using namespace cv;
using namespace std;

const int w = 500;
int levels = 3;
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
static void on_trackbar(int, void*)
{
    Mat cnt_img = Mat::zeros(w, w, CV_8UC3);
    int _levels = levels - 3;
    drawContours( cnt_img, contours, 0, Scalar(128,255,255),
                  3, LINE_AA, hierarchy, std::abs(_levels) );
    imshow("contours", cnt_img);
}

//https://answers.opencv.org/question/59782/how-to-detect-contours-for-two-or-more-than-two-specific-colors/

int main( int argc, char** argv)
{

    Mat img = Mat::zeros(w, w, CV_8UC1);
    //Draw 6 faces
    for( int i = 0; i < 1; i++ )
    {
        int dx = (i%2)*250 - 30;
        int dy = (i/2)*150;
        const Scalar white = Scalar(255);
        const Scalar black = Scalar(0);
        
        /*
        ellipse( img, Point(dx+150, dy+100), Size(100,70), 0, 0, 360, white, -1, 8, 0 );
        */
        rectangle(img, Point(50, 50), Point(100, 100), white, -1);
        rectangle(img, Point(10, 70), Point(210, 90), white, -1);
        rectangle(img, Point(150, 50), Point(200, 100), white, -1);

 /*     ellipse( img, Point(dx+115, dy+70), Size(30,20), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+185, dy+70), Size(30,20), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+115, dy+70), Size(15,15), 0, 0, 360, white, -1, 8, 0 );
        ellipse( img, Point(dx+185, dy+70), Size(15,15), 0, 0, 360, white, -1, 8, 0 );
        ellipse( img, Point(dx+115, dy+70), Size(5,5), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+185, dy+70), Size(5,5), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+150, dy+100), Size(10,5), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+150, dy+150), Size(40,10), 0, 0, 360, black, -1, 8, 0 );
        ellipse( img, Point(dx+27, dy+100), Size(20,35), 0, 0, 360, white, -1, 8, 0 );
        ellipse( img, Point(dx+273, dy+100), Size(20,35), 0, 0, 360, white, -1, 8, 0 );*/
    }
    //show the faces
    namedWindow( "image", 1 );
    imshow( "image", img );
    //Extract the contours so that
    vector<vector<Point> > contours0;
    findContours( img, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    contours.resize(contours0.size());
    for (size_t k = 0; k < contours0.size(); k++)
    {
        yarp::dev::Map2DArea area;
        area.map_id = "test";
        approxPolyDP(Mat(contours0[k]), contours[k], 4, true);
        for (size_t kk = 0; kk < contours[k].size(); kk++)
        {
            cout << k << " " << kk << " x:" << contours[k][kk].x << " y:" << contours[k][kk].y << endl;
            area.points.push_back(yarp::math::Vec2D<double>(contours[k][kk].x, contours[k][kk].y));
        }
    }

    namedWindow( "contours", 1 );
    createTrackbar( "levels+3", "contours", &levels, 7, on_trackbar );
    on_trackbar(0,0);
    waitKey();
    return 0;
}