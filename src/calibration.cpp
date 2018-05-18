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

// 2.6 cm 26 mm 0.026
const float calibrationSquareDimension = 0.026f; //meters
const float arucoSquareDimension = 0.15f; //meters
const Size chessboardDimensions = Size(6,9);

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

void createArucoMarkers()
{
    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    for(int i = 0; i < 50 ; i++)
    {
      aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
      ostringstream convert;
      string imageName = "4X4_50_Marker_";
      convert << imageName << i << ".jpg";
      imwrite(convert.str(), outputMarker);
    }
}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
      for(int i = 0; i < boardSize.height; i++)
      {
        for(int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength, 0.0f));
        }
      }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f > >& allFoundCorners, bool showResults = false)
{

      for (vector<Mat>::iterator inter = images.begin(); inter != images.end(); inter++)
      {
          vector<Point2f> pointBuf;
          bool found = findChessboardCorners(*inter, Size(9,6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
          
          if(found)
          {
            allFoundCorners.push_back(pointBuf);
          }

          if(showResults)
          {
            drawChessboardCorners(*inter, Size(6, 9), pointBuf, found);
            imshow("Looking for Corners", *inter);
            waitKey(0);
          }

      }
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients)
{
  vector<vector<Point2f > > cheackboardImageSpacePoints;
  getChessboardCorners(calibrationImages, cheackboardImageSpacePoints, false);

  vector<vector<Point3f > > worldSpaceCornerPoints(1);

  createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(cheackboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);

  calibrateCamera(worldSpaceCornerPoints, cheackboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);

}

bool saveCameraCalibration(Mat cameraMatrix, Mat distanceCoefficients)
{

  ofstream myfile;

  myfile.open("../calibrationFile.txt");
  if (myfile.is_open())
  {
      uint16_t rows = cameraMatrix.rows;
      uint16_t columns = cameraMatrix.cols;

      myfile << rows << endl;
      myfile << columns << endl;

      for( int r = 0; r < rows; r++)
      {
        for( int c = 0; c < columns; c++)
        {
          double value = cameraMatrix.at<double>(r, c);
          myfile << value << endl;
        }
      }

      rows = distanceCoefficients.rows;
      columns = distanceCoefficients.cols;

      myfile << rows << endl;
      myfile << columns << endl;

      for (int r = 0; r < rows; r++)
      {
        for (int c = 0; c < columns; c++)
        {
          double value = distanceCoefficients.at<double>(r, c);
          myfile << value << endl;
        }
      }
    myfile.close();
    return true;
  }

  return false;
}

int main(int argc, char const *argv[])
{

  Mat frame, drawToFrame, distanceCoefficients;

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);

  vector<Mat> savedImages;

  vector<vector<Point2f > > markerCorners, rejectedCandidates;

  int framePerSecond = 80;

  int cont = 0;

  VideoCapture video(1);

  cout << "*************Camerea Calibration*************" << endl;
  cout << "Recomended: https://www.youtube.com/watch?v=HNfPbw-1e_w&index=14&list=PLAp0ZhYvW6XbEveYeefGSuLhaPlFML9gP" << endl;
  cout << "You have to print out the file 'pattern.png' " << endl;
  cout << "Options of the program:" << endl;
  cout << "Press the key 'space' to save the images to calibration" << endl;
  cout << "Press the key 'e' to save the images calibrated into notebook" << endl;
  cout << "Press the key 'esc' to exit the program" << endl;


  //add the fps controll
  cout << "fps: " << framePerSecond << endl;

  if (!video.isOpened())
  {
    cout << "Camera was not opened" << endl;
    return -1;
  }

  namedWindow("WebCam Calibration", CV_WINDOW_AUTOSIZE);

  // //set height and width of capture frame
  // if (video.isOpened())
  // {
  //   video.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  //   video.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  // }

  while (true)
  {
    if (!video.read(frame))
    {
      break;
    }

    video.read(frame);

    vector<Vec2f> foundPoints;
    bool found = false;

    found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(drawToFrame);

    drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

    if(found)
    {
      //cout << "found " << endl;
      imshow("WebCam Calibration", drawToFrame);
    } else {
      //cout << "not found " << endl;
      imshow("WebCam Calibration", frame);
    }
    char character = waitKey(1000 / framePerSecond);

    switch(character)
    {
      //'espace ' has been pressed. this will save the images to calibration
      case ' ':
        //save image
        if(found)
        {
          Mat temp;
          frame.copyTo(temp);
          savedImages.push_back(temp);
          cont++;
          cout << "The Image [" << cont << "] was saved!" << endl;
          if(cont == 30)
          {
            cout << "Your calibrate is good to be saved! with " << cont << " images" << endl;
          }
        }
        break;
      //'e' has been pressed. this will save the images calibrated into notebook
      case 101:
        cout << "Total of images saved: " << savedImages.size() << endl;
        //start calibration
        if(savedImages.size() > 15)
        {
          cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
          saveCameraCalibration(cameraMatrix, distanceCoefficients);
          cout << "The Images Calibrated was saved!" << endl;
        }
        break;
      case 27:
        //exit
        return 0;
        break;
    }

  }

  return 0;
}