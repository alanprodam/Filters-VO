#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/features2d/features2d.hpp"

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <ctype.h>
#include <algorithm>
#include <iterator>
#include <ctime>
#include <string>
#include <stdio.h>

using namespace std;
using namespace cv;

// 2.6 cm - 26 mm - 0.026 m
const float calibrationSquareDimension = 0.026f; //meters
// 13.2 cm - 132 mm - 0.132 m
// 33.9 cm - 339 mm - 0.339 m
const float arucoSquareDimensionMaior = 0.339f; //meters
// 7.9 cm - 79 mm - 0.079 m
const float arucoSquareDimensionMenor = 0.079f; //meters

// dimension of cheesboard
const Size chessboardDimensions = Size(6,9);

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//initial min and max THRESHOLD filter values
//our sensitivity value to be used in the threshold() function
//these will be changed using trackbars
int THRESHOLD_MIN = 220;
int THRESHOLD_MAX = 256;
int BLUR_SIZE = 5;
int DILATATION = 2;
int ERODE = 2;

//diameter of circle is 54 cm
const float DIAMETER = 54;

// focal distance
const float FOCO = 675;

void on_trackbar(int, void *)
{ //This function gets called whenever a
  // trackbar position is changed
}

//int to string helper function
string intToString(int number)
{
  //this function has a number input and string output
  std::stringstream ss;
  ss << number;
  return ss.str();
}

//float to string helper function
string floatToString(float number)
{
  //this function has a number input and string output
  std::stringstream ss;
  ss << number;
  return ss.str();
}

string doubleToString(double number)
{
  //this function has a number input and string output
  std::stringstream ss;
  ss << number;
  return ss.str();
}

float distanceLandmarck(int &radius)
{
  // f = foco(taxa de transformação); x = diâmetro(pixels);
  // Z = distáncia entre câmera(altura) = landmark(centimetros); X = diâmetro do landmark(centimetros)
  // Z = (X * f) / x;

  // diâmetro circulo em pixels
  float pixelDiametro = radius * 2;

  // f(24) = 672 f(20) = 680 f(54) = 675
  float altura = (DIAMETER * FOCO) / pixelDiametro;

  return altura;
}

// calcular distancias(x,y) do mundo real entre landmark e o frame do drone
float *positionCenter(float &xLandmark, float &yLandmark, float &altura)
{
  float px = xLandmark - FRAME_WIDTH / 2;

  float py = yLandmark - FRAME_HEIGHT / 2;

  float *out = (float *)calloc(2, sizeof(float));

  // contastane k que relaciona diametro com a quantidade de pixels Z/f ou X/x

  float k = altura / FOCO;

  out[0] = px * k;

  out[1] = py * k;

  return out;
}

void searchForCircule(Mat &thresholdImage, Mat &cameraFeed)
{
  vector<Vec3f> circles;
  HoughCircles(thresholdImage, circles, HOUGH_GRADIENT, 1.4,
               thresholdImage.rows / 2, // change this value to detect circles with different distances to each other
               100, 100, 0, 0           // change the last two parameters
                                        // (min_radius & max_radius) to detect larger circles
  );

  //cout << "size circulo " << circles.size() << endl;
  if (circles.size() == 1)
  {
    for (size_t i = 0; i < circles.size(); i++)
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

      int radius = cvRound(circles[i][2]);
      float Xc = cvRound(circles[i][0]);
      float Yc = cvRound(circles[i][1]);

      float Altura = distanceLandmarck(radius);

      // converction cm to mm
      Altura = Altura * 10;

      float *direction = positionCenter(Xc, Yc, Altura);

      // converction to meters
      Altura = Altura / 1000;

      // circle center
      circle(cameraFeed, center, 3, Scalar(0, 255, 0), -1, 8, 0);
      // circle outline
      circle(cameraFeed, center, radius, Scalar(0, 0, 255), 3, 8, 0);
      putText(cameraFeed, "Tracking first aim at (" + intToString(cvRound(circles[i][0])) + "pixel , " + intToString(cvRound(circles[i][1])) + "pixel )",
              Point(10, 20), 1.2, 1.2, Scalar(255, 0, 0), 2);
      putText(cameraFeed, "Altitude: " + doubleToString(Altura) + " m",
              Point(10, 50), 1.2, 1.2, Scalar(0, 0, 255), 2);
      putText(cameraFeed, "Correction coordinates (" + floatToString(cvRound(direction[0])) + "mm , " + floatToString(cvRound(direction[1])) + "mm )",
              Point(10, 80), 1.2, 1.2, Scalar(0, 255, 0), 2);
      putText(cameraFeed, "X",
              Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);

      // Vec3i c = circles[i];
      // circle(cameraFeed, Point(c[0], c[1]), c[2], Scalar(0, 0, 255), 3, LINE_AA);
      // circle(cameraFeed, Point(c[0], c[1]), 2, Scalar(0, 255, 0), 3, LINE_AA);
    }

    // realizar o processo para identificar sempre o maior circulo para calculo da altura
    // associar os circulos (diametro pixels) com suas respectivas distancias
    // altura próxima de 1 m baixar o threshold
    // achar triangulo para encontrar orientação (por angulos em relação ao quadrado)
  }
}

bool loadCameraCalibration(Mat& cameraMatrix, Mat& distanceCoefficients)
{
  ifstream myfile;

  myfile.open("calibrationFile(smartphone).txt");
  if (myfile.is_open())
  {
    uint16_t rows;
    uint16_t columns;

    myfile >> rows;
    myfile >> columns;

    cameraMatrix = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {

        double read = 0.0f;
        myfile >> read;
        cameraMatrix.at<double>(r, c) = read;
        //cout << cameraMatrix.at<double>(r, c) << endl;
      }
    }

    //Distance Coefficients
    myfile >> rows;
    myfile >> columns;

    distanceCoefficients = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++)
    {
      for (int c = 0; c < columns; c++)
      {
        double read = 0.0f;
        myfile >> read;
        distanceCoefficients.at<double>(r, c) = read;
        //cout << distanceCoefficients.at<double>(r, c) << endl;
      }
    }
    myfile.close();
    return true;
  }

  return false;
}

int startWebcamMonitoring(Mat& cameraMatrix, Mat& distanceCoefficients, float arucoSquareDimension)
{
  Mat frame, frameCopy;

  vector<int> markerIds;
  vector<vector<Point2f > > markerCorners, rejectedCandidates;

  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary>  markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

  VideoCapture video(1);

  vector<Vec3d> rotationVectors, translationVectors;

  // visualizar parametros intrinsecos da camera
  for (int r = 0; r < cameraMatrix.rows; r++)
  {
    for (int c = 0; c < cameraMatrix.cols; c++)
    {
      cout << cameraMatrix.at<double>(r, c) << endl;
    }
  }

  // visualizar distorções da camera
  for (int r = 0; r < distanceCoefficients.rows; r++)
  {
    for (int c = 0; c < distanceCoefficients.cols; c++)
    {
      cout << distanceCoefficients.at<double>(r, c) << endl;
    }
  }

  namedWindow("WebCam Aruco", CV_WINDOW_NORMAL);

  while (1)
  {

    video.open("/home/alantavares/Filters/Dataset-Filters/indoor.mp4");

    if (!video.isOpened())
    {
      cout << "ERROR ACQUIRING VIDEO FEED" << endl;
      getchar();
      return -1;
    }
    else{
      cout << "CORRECT ACQUIRING VIDEO FEED" << endl;
    }

    //check if the video has reach its last frame.
    //we add '-1' because we are reading two frames from the video at a time.
    //if this is not included, we get a memory error!
    while (video.get(CV_CAP_PROP_POS_FRAMES) < video.get(CV_CAP_PROP_FRAME_COUNT) - 1)
    {

      //read first frame
      video.read(frame);

      frame.copyTo(frameCopy);

      aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
      aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix,
                                       distanceCoefficients, rotationVectors, translationVectors);

      //cout << "markerIds: " << markerIds.size() << endl;

      for (int i = 0; i < markerIds.size(); i++)
      {
        //cout << "markerCorners: " << markerCorners.at<Point2f> << endl;
        aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);

      }

      //*****************************************************************

      // //thresholded difference image (for use in findContours() function)
      // Mat thresholdImage, grayImage;

      // //convert frame1 to gray scale for frame differencing
      // cvtColor(frameCopy, grayImage, COLOR_BGR2GRAY);

      // //threshold intensity image at a given sensitivity value
      // threshold(grayImage, thresholdImage, THRESHOLD_MIN, THRESHOLD_MAX, THRESH_BINARY);

      // //use blur() to smooth the image, remove possible noise and
      // blur(thresholdImage, thresholdImage, Size(BLUR_SIZE, BLUR_SIZE));

      // searchForCircule(thresholdImage,frame);

      imshow("WebCam Aruco", frame);
      if (waitKey(1) >= 0)
      {
        break;
      }
    }
  return 1;
  }
}

int main(int argc, char const *argv[])
{

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  loadCameraCalibration(cameraMatrix, distanceCoefficients);
  startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimensionMaior);
  
  return 0;
}