#include "libopencv.h"
#include "libSquares.h"
// #include "libCircles.h"
#include <stdio.h>

using namespace cv;
using namespace std;

#define MAX_FRAME 375

int main( int argc, char** argv ) {

  Mat img_c, img_resize, img_gray, imblur, im_canny;

  char filename[200];



  for(int i=150;i<MAX_FRAME;i++){

      sprintf(filename, "/home/alantavares/landing-filter/Dataset-no-drone-perfect/%03d.jpg",i);

      img_c = imread(filename);
      if ( !img_c.data){
        std::cout<< " --(!) Error Leitura Imagem " << std::endl; return -1;
      }

      resize(img_c,img_resize,cv::Size(img_c.cols/3,img_c.rows/3));
      //cout << "Dimensão images: "<<  img_resize.cols <<"x"<< img_resize.rows << endl;

      vector<vector<Point> > squares;
      findSquares(img_resize, squares);
      drawSquares(img_resize, squares);



      //![convert_to_gray]
      cvtColor(img_resize, img_gray, COLOR_BGR2GRAY);
      //![convert_to_gray]

      //![reduce_noise]
      GaussianBlur(img_gray, imblur,Size(3,3),5,5,0);
      //![reduce_noise]

      //![bords]
      //Canny(imblur, im_canny, 30, 80, 3);
      //![bords]

      int min = 200;
      int max = 255;

      Mat limiarizada;
      threshold(img_gray,limiarizada,min,max,CV_THRESH_BINARY);          
      //threshold(img_gray,limiarizada.at(3),min,max,CV_THRESH_TOZERO);      

      //imshow("LimiarBin",limiarizada.at(0));
      imshow("LimiarToZero",limiarizada);

      //![houghcircles]
      // vector<Vec3f> circles;
      // HoughCircles(imblur, circles, HOUGH_GRADIENT, 1,
      //              imblur.rows/11, // change this value to detect circles with different distances to each other
      //              100, 30, 1, 200 // change the last two parameters
      //                             // (min_radius & max_radius) to detect larger circles
      //              );
      //![houghcircles]

      //![draw]
      // for( size_t i = 0; i < circles.size(); i++ )
      // {
      //     Vec3i c = circles[i];
      //     circle( img_resize, Point(c[i], c[i+1]), c[2], Scalar(0,0,255), 3, LINE_AA); //red ao redor
      //     circle( img_resize, Point(c[i], c[i+1]), 1, Scalar(0,255,0), 3, LINE_AA); // green centro
      // }
      //![draw]

      //imshow("Imagem" , im_canny);

      //![display]
      //imshow("detected circles", img_resize);

      waitKey(100);

    }
 
  return 0;
}