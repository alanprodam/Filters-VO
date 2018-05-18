#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm>
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <stdio.h>

using namespace cv;
using namespace std;

const int fps = 30;

//initial min and max THRESHOLD filter values
//our sensitivity value to be used in the threshold() function
//these will be changed using trackbars
int THRESHOLD_MIN = 220;
int THRESHOLD_MAX = 256;
int DILATATION = 2;
int ERODE = 2;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//diameter of circle is 54 cm
const float DIAMETER = 54;

// focal distance
const float FOCO = 675;

//names that will appear at the top of each window
const string trackbarWindowName = "Trackbars";

//size of blur used to smooth the image to remove possible noise and
//increase the size of the object we are trying to track. (Much like dilate and erode)
int BLUR_SIZE = 5;

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

void createTrackbars()
{
  //create window for trackbars

  namedWindow(trackbarWindowName, CV_WINDOW_FREERATIO);
  
  //create memory to store trackbar name on window
  char TrackbarName[50];
  sprintf(TrackbarName, "THRESHOLD_MIN");
  sprintf(TrackbarName, "THRESHOLD_MAX");
  sprintf(TrackbarName, "BLUR_SIZE");
  sprintf(TrackbarName, "DILATATION");
  sprintf(TrackbarName, "ERODE");
  //create trackbars and insert them into window
  //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
  //the max value the trackbar can move (eg. H_HIGH),
  //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
  //                                  ---->    ---->     ---->
  createTrackbar("THRESHOLD_MIN", trackbarWindowName, &THRESHOLD_MIN, THRESHOLD_MAX, on_trackbar);
  createTrackbar("THRESHOLD_MAX", trackbarWindowName, &THRESHOLD_MAX, THRESHOLD_MAX, on_trackbar);
  createTrackbar("BLUR_SIZE", trackbarWindowName, &BLUR_SIZE, 50, on_trackbar);
  createTrackbar("DILATATION", trackbarWindowName, &DILATATION, 30, on_trackbar);
  createTrackbar("ERODE", trackbarWindowName, &ERODE, 30, on_trackbar);
}

float distanceLandmarck(int &radius)
{
  // f = foco(taxa de transformação); x = diâmetro(pixels);
  // Z = distáncia entre câmera(altura) = landmark(centimetros); X = diâmetro do landmark(centimetros)
  // Z = (X * f) / x;

  // diâmetro circulo em pixels
  float pixelDiametro = radius * 2;

  // 24cm - 20cm - 54cm - diâmetro circulo em centimetros
  //double X = 54;

  // f(24) = 672 f(20) = 680 f(54) = 675
  float altura = (DIAMETER * FOCO) / pixelDiametro;
  //if (x > 170 && x < 300)
  //if (x < 170)
  //if (x > 300)
  // if (pixelDiametro != 0)
  // {
  //   cout << " foco: " << foco
  //        << " Diametro(px): " << pixelDiametro
  //        << " Diametro(cm): " << centDiameter
  //        << " Altura(cm): " << altura << endl;
  // }

  return altura;
}

// calcular distancias(x,y) do mundo real entre landmark e o frame do drone
float * positionCenter(float &xLandmark, float &yLandmark, float &altura)
{
  float px = xLandmark - FRAME_WIDTH / 2;

  float py = yLandmark - FRAME_HEIGHT / 2;

  float *out = (float *) calloc (2, sizeof(float));

  // contastane k que relaciona diametro com a quantidade de pixels Z/f ou X/x

  float k = altura/FOCO;

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

// calibração da taxa de transformação
double distFocus(int &radius, double &Z, double &X)
{
  // f = foco(taxa de transformação); x = diâmetro(pixels);
  // Z = distáncia entre câmera - landmark(centimetros); X = diâmetro do landmark(centimetros)
  // f = (x * Z) / X;

  double x = radius * 2;
  // 80cm

  //double Z = 84;
  
  // 24cm - 20cm - 54cm
  //double X = 24;

  // f(24) = 672 f(20) = 680 f(54) = 675
  double f = (x * Z) / X;

  //if (x > 170 && x < 300) - circulo médio
  //if (x < 170) - circulo menor
  //if (x > 300) - circulo maior
  if (x > 170 && x < 300)
  {
    cout << " Diametro(px): " << x
         << " Diametro(cm): " << X
         << " foco: " << f << endl;
  }

  return f;
}

int main(int argc, char const *argv[])
{

  //create slider bars for HSV filtering
  createTrackbars();

  bool windowCreated = false;
  bool windowCreated2 = false;
  bool windowCreated3 = false;

  //some boolean variables for added functionality
  bool objectDetected = false;

  //these two can be toggled by pressing 'd'
  bool debugMode = false;
  //these two can be toggled by pressing 'e'dd
  bool debugMode2 = false;
  //these two can be toggled by pressing 'o'
  bool debugMode3 = false;

  //these two can be toggled by pressing 't'
  bool trackingEnabled = false;
  //pause and resume code
  bool pause = false;
  //set up the matrices that we will need
  //the two frames we will be comparing
  Mat frame1, frame2;
  //their grayscale images (needed for absdiff() function)
  Mat grayImage1, grayImage2;
  //resulting difference image
  Mat differenceImage;
  //thresholded difference image (for use in findContours() function)
  Mat thresholdImage;
  //video capture object.
  VideoCapture capture(1);

  //open capture object at location zero (default location for webcam)
  //capture.open(1);
  capture.open("/home/alantavares/Filters/Dataset-Filters/dataset-estavel.mp4");

  //add the fps controll
  cout << "fps: " << fps << endl;
  cout << "entrou f1" << endl;

  if (!capture.isOpened())
  {
    cout << "Camera was not opened" << endl;
    return -1;
  }

  // //set height and width of capture frame
  // if (capture.isOpened())
  // {
  //   capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  //   capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  // }

  //begin of video read
  while (1)
  {
    clock_t begin = clock();

    //read first frame
    capture.read(frame1);

    //convert frame1 to gray scale for frame differencing
    cvtColor(frame1, grayImage1, COLOR_BGR2GRAY);

    //threshold intensity image at a given sensitivity value
    threshold(grayImage1, thresholdImage, THRESHOLD_MIN, THRESHOLD_MAX, THRESH_BINARY);

    if (debugMode == true)
    {
      namedWindow("Threshold Image", CV_WINDOW_AUTOSIZE);
      imshow("Threshold Image", thresholdImage);

      if (trackingEnabled)
      {
        //searchForMovement(thresholdImage, frame1);
        searchForCircule(thresholdImage, frame1);
      }

      windowCreated = true;
    }
    else
    {
      if (windowCreated)
      {
        destroyWindow("Threshold Image");

        windowCreated = false;
      }
    }

    if (BLUR_SIZE > 0)
    {
      //use blur() to smooth the image, remove possible noise and
      blur(thresholdImage, thresholdImage, Size(BLUR_SIZE, BLUR_SIZE));
    }

    //threshold again to obtain binary image from blur output
    //threshold(thresholdImage, thresholdImage, 250, 255, THRESH_BINARY);

    if (debugMode2 == true)
    {
      namedWindow("Blur Image", CV_WINDOW_AUTOSIZE);
      imshow("Blur Image", thresholdImage);

      if (trackingEnabled)
      {
        //searchForMovement(thresholdImage, frame1);
        searchForCircule(thresholdImage, frame1);
      }

      windowCreated2 = true;
    }
    else
    {
      if (windowCreated2)
      {
        destroyWindow("Blur Image");

        windowCreated2 = false;
      }
    }

    //increase the size of the object we are trying to track. (Much like dilate and erode)
    //dilate with larger element so make sure object is nicely visible
    if (ERODE > 0)
    {
      Mat erodeElement = getStructuringElement(MORPH_RECT, Size(ERODE, ERODE));
      erode(thresholdImage, thresholdImage, erodeElement);
    }

    if (DILATATION > 0)
    {
      Mat dilateElement = getStructuringElement(MORPH_RECT, Size(DILATATION, DILATATION));
      dilate(thresholdImage, thresholdImage, dilateElement);
    }

    //threshold(thresholdImage, thresholdImage, 200, 255, THRESH_BINARY);

    if (debugMode3 == true)
    {
      imshow("Morf Image", thresholdImage);

      if (trackingEnabled)
      {
        searchForCircule(thresholdImage, frame1);
      }

      windowCreated3 = true;
    }
    else
    {
      if (windowCreated3)
      {
        destroyWindow("Morf Image");

        windowCreated3 = false;
      }
    }

    //show our captured frame
    namedWindow("Frame1", CV_WINDOW_AUTOSIZE);
    imshow("Frame1", frame1);

    //resizeWindow("Frame1", frame1.cols/2, frame1.rows/2);
    //moveWindow("Frame1", 400, 0);

    // if (waitKey(1000 / fps) >= 0)
    // {
    //   break;
    // }

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    //cout << "Time taken: " << elapsed_secs << "s" << endl;
    //cout << "fps taken: " << (1/elapsed_secs) << "fps" << endl;

    switch (waitKey(10))
    {

    case 27: //'esc' key has been pressed, exit program.
      return 0;
    case 116: //'t' has been pressed. this will toggle tracking
      trackingEnabled = !trackingEnabled;
      if (trackingEnabled == false)
        cout << "Tracking disabled." << endl;
      else
        cout << "Tracking enabled." << endl;
      break;
    case 100: //'d' has been pressed. this will debug mode
      debugMode = !debugMode;
      if (debugMode == false)
        cout << "Debug mode disabled." << endl;
      else
        cout << "Debug mode enabled." << endl;
      break;
    case 101: //'e' has been pressed. this will debug mode
      debugMode2 = !debugMode2;
      if (debugMode2 == false)
        cout << "Debug2 mode disabled." << endl;
      else
        cout << "Debug2 mode enabled." << endl;
      break;
    case 111: //'o' has been pressed. this will debug mode
      debugMode3 = !debugMode3;
      if (debugMode3 == false)
        cout << "Debug3 mode disabled." << endl;
      else
        cout << "Debug3 mode enabled." << endl;
      break;
    case 112: //'p' has been pressed. this will pause/resume the code.
      pause = !pause;
      if (pause == true)
      {
        cout << "Code paused, press 'p' again to resume" << endl;
        while (pause == true)
        {
          //stay in this loop until
          switch (waitKey())
          {
            //a switch statement inside a switch statement? Mind blown.
          case 112:
            //change pause back to false
            pause = false;
            cout << "Code resumed." << endl;
            break;
          }
        }
      }
    }
  }

  return 0;
  }
