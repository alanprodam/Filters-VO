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
const float calibrationSquareDimension = 0.025f; //meters
// 13.2 cm - 132 mm - 0.132 m
// 33.9 cm - 339 mm - 0.339 m
const float arucoSquareDimensionMaior = 0.1325f; //meters
const Size chessboardDimensions = Size(6,9);

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

void createArucoMarkers()
{
    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

    for(int i = 0; i < 1000 ; i++)
    {
      aruco::drawMarker(markerDictionary, i, 400, outputMarker, 1);
      ostringstream convert;
      string imageName = "4X4_1000_Marker_";
      convert << imageName << i << ".jpeg";
      imwrite(convert.str(), outputMarker);
    }
}

bool loadCameraCalibration(Mat& cameraMatrix, Mat& distanceCoefficients)
{
  ifstream myfile;

  myfile.open("calibrationFile.txt");
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
  Mat frame, imageCopy;

  vector<int> markerIds;
  vector<vector<Point2f > > markerCorners, rejectedCandidates;

  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_1000);

  VideoCapture video(1);

  if (!video.isOpened())
  {
    cout << "Camera was not opened" << endl;
    return -1;
  }

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

  namedWindow("WebCam Aruco", CV_WINDOW_AUTOSIZE);

  while (true)
  {
    if (!video.read(frame))
    {
      break;
    }

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, 
      distanceCoefficients, rotationVectors, translationVectors);

    //frame.copyTo(imageCopy);

    //cout << "markerIds: " << markerIds.size() << endl;

    for (int i = 0; i < markerIds.size(); i++)
    {
      //if (markerIds[i] == 269)
      //{
        //cout << " marcador: " << markerIds[i] << endl;
        aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
        cout << " translation [" << markerIds[i] << "] = " << translationVectors[i] << " ; rotation:" << rotationVectors[i] << endl;
      //}

    }

    putText(frame, "X", Point(FRAME_WIDTH / 2, FRAME_HEIGHT / 2), 1.2, 1.2, Scalar(0, 0, 255), 2);

    imshow("WebCam Aruco", frame);
    if (waitKey(1) >= 0)
    {
      break;
    }
  }
  return 1;
}

int main(int argc, char const *argv[])
{
  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  //createArucoMarkers();
  loadCameraCalibration(cameraMatrix, distanceCoefficients);
  startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimensionMaior);

  return 0;
}