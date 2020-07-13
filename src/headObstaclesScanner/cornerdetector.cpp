#include "cornerdetector.h"



using namespace cv;
using namespace std;
using namespace cv;
using namespace std;
using namespace yarp::sig;
using namespace yarp::os;




cornerDetector::cornerDetector(std::string m_map_name)
{
    map_name = m_map_name;
}

void cornerDetector::calculateCorners()
{
    /// Load source image and convert it to gray
    src = cv::imread( map_name, 1 );

    cvtColor( src, src_gray, CV_BGR2GRAY );

    trackFeatures();

    yarp_corners.resize(corners.size() , 2);

    for( int i = 0; i < corners.size(); i++ )
    {
        yarp_corners(i,0) = corners[i].x;
        yarp_corners(i,1) = corners[i].y;
    }


}

void cornerDetector::trackFeatures()
{

  qualityLevel = qualityLevelTB*1.0 / 10000;
  if( qualityLevel <= 0 ) { qualityLevel = 0.001; }
  if( blockSize <= 0 ) { blockSize = 1; }
  if( maxCorners < 1 ) { maxCorners = 1; }

  std::cout << "quality level: " << qualityLevel << '\n';

  rng(12345);

  /// Parameters for Shi-Tomasi algorithm

  bool useHarrisDetector = false;
  double k = 0.04;

  /// Copy the source image
  Mat copy;
  copy = src.clone();

  /// Apply corner detection
  goodFeaturesToTrack( src_gray,
               corners,
               maxCorners,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );

  /// Plot first corner coordinate
//  cout << "X[0]: " << corners[0].x << " Y[0]: " << corners[0].y << '\n';
//  cout << "X[1]: " << corners[1].x << " Y[1]: " << corners[1].y << '\n';

}
