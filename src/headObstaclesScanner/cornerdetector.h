#ifndef CORNERDETECTOR_H
#define CORNERDETECTOR_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <string>
#include <cstdio>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>



#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;
using namespace yarp::sig;
using namespace yarp::os;


class cornerDetector
{
public:
    cornerDetector(std::string m_map_name);

    // VARIABLES
    yarp::sig::Matrix yarp_corners;
    vector<Point2f> corners;

    // METHODS
    void calculateCorners(void);

private:

    // VARIABLES
    Mat src, src_gray;
    std::string map_name;

    int maxTrackbarCor = 1000;
    int maxTrackbarDist = 100;
    int maxTrackbarQl = 10000*0.01*100;
    int maxTtackbarBs = 3*10;

    int maxCorners = 502;
    double qualityLevel = 0.46;
    int minDistance = 11;
    int blockSize = 3;

    int qualityLevelTB = qualityLevel * 10000;

    cv::RNG rng;

    char* source_window = "Image";


    // METHODS

    void trackFeatures(void);




};

#endif // CORNERDETECTOR_H
