#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void findCircles( const Mat& image)
{   
    //![convert_to_gray]
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    //![convert_to_gray]

    //![reduce_noise]
    medianBlur(gray, gray, 5);
    //![reduce_noise]

    vector<Vec3f> circles;
    //![houghcircles]
    HoughCircles(image, circles, HOUGH_GRADIENT, 1,
                 image.rows/16, // change this value to detect circles with different distances to each other
                 100, 30, 1, 30 // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );
    //![houghcircles]

    //![draw]
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( image, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, LINE_AA);
        circle( image, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, LINE_AA);
    }
    //![draw]
}
