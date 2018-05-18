#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>
#include <sstream>
#include <string>

using namespace cv;
using namespace std;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 254;
int V_MAX = 256;
int DIL = 3;
int ERO = 6;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 50;

//minimum and maximum object area
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

void on_trackbar(int, void *)
{ //This function gets called whenever a
  // trackbar position is changed
}
string intToString(int number)
{

  std::stringstream ss;
  ss << number;
  return ss.str();
}

void createTrackbars()
{
  //create window for trackbars

  namedWindow(trackbarWindowName, 0);
  //create memory to store trackbar name on window
  char TrackbarName[50];
  sprintf(TrackbarName, "H_MIN");
  sprintf(TrackbarName, "H_MAX");
  sprintf(TrackbarName, "S_MIN");
  sprintf(TrackbarName, "S_MAX");
  sprintf(TrackbarName, "V_MIN");
  sprintf(TrackbarName, "V_MAX");
  sprintf(TrackbarName, "DIL");
  sprintf(TrackbarName, "ERO");
  //create trackbars and insert them into window
  //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
  //the max value the trackbar can move (eg. H_HIGH),
  //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
  //                                  ---->    ---->     ---->
  createTrackbar("H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar);
  createTrackbar("H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar);
  createTrackbar("S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar);
  createTrackbar("S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar);
  createTrackbar("V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar);
  createTrackbar("V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar);
  createTrackbar("DIL", trackbarWindowName, &DIL, 20, on_trackbar);
  createTrackbar("ERO", trackbarWindowName, &ERO, 20, on_trackbar);
}

void drawObject(int x, int y, Mat &frame)
{

  //use some of the openCV drawing functions to draw crosshairs
  //on your tracked image!

  //UPDATE:JUNE 18TH, 2013
  //added 'if' and 'else' statements to prevent
  //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

  circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
  if (y - 25 > 0)
    line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
  else
    line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
  if (y + 25 < FRAME_HEIGHT)
    line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
  else
    line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
  if (x - 25 > 0)
    line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
  else
    line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
  if (x + 25 < FRAME_WIDTH)
    line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
  else
    line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

  putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
}
void morphOps(Mat &thresh)
{
  //create structuring element that will be used to "dilate" and "erode" image.
  //the element chosen here is a 3px by 3px rectangle

  //use blur() to smooth the image, remove possible noise and
  blur(thresh, thresh, Size(10, 10));

  if (DIL > 1)
  {
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement(MORPH_RECT, Size(DIL, DIL));
    dilate(thresh, thresh, dilateElement);
  }

  if (ERO > 1)
  {
    Mat erodeElement = getStructuringElement(MORPH_RECT, Size(ERO, ERO));
    erode(thresh, thresh, erodeElement);
  }
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed)
{

  Mat temp;
  threshold.copyTo(temp);
  //these two vectors needed for output of findContours
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;

  //find contours of filtered image using openCV findContours function
  findContours(temp, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

  vector<Point> approx;

  cout << " contours " << contours.size() << endl;

  if (contours.size() > 0)
  {

    int numObjects = contours.size();

    if (numObjects < 50)
    {
      for (size_t index = 0; index < contours.size(); index++)
      {
        double perimetro = arcLength(Mat(contours[index]), true);
        approxPolyDP(Mat(contours[index]), approx, perimetro * 0.02, true);

        
        
         if (approx.size() < 3 && isContourConvex(Mat(approx)))
        {

          

        }
    
      }

    } else {
      putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
    }
       
  }
}

int main(int argc, char *argv[])
{
  //some boolean variables for different functionality within this
  //program
  bool trackObjects = true;
  bool useMorphOps = true;

  //Matrix to store each frame of the webcam feed
  Mat cameraFeed;
  //matrix storage for HSV image
  Mat HSV;
  //matrix storage for binary threshold image
  Mat threshold;

  //x and y values for the location of the object
  int x = 0, y = 0;
  //create slider bars for HSV filtering
  createTrackbars();
  //video capture object to acquire webcam feed
  VideoCapture capture;
  //open capture object at location zero (default location for webcam)
  capture.open(0);

  //set height and width of capture frame
  capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
  //start an infinite loop where webcam feed is copied to cameraFeed matrix
  //all of our operations will be performed within this loop
  while (1)
  {
    //store image to matrix
    capture.read(cameraFeed);

    //convert frame from BGR to HSV colorspace
    cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);

    //filter HSV image between values and store filtered image to
    //threshold matrix
    inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if (useMorphOps)
    {
      morphOps(threshold);
    }

    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if (trackObjects)
    {
      trackFilteredObject(x, y, threshold, cameraFeed);
    }

    //show frames
    imshow(windowName2, threshold);
    imshow(windowName, cameraFeed);
    imshow(windowName1, HSV);

    //delay 30ms so that screen can refresh.
    //image will not appear without this waitKey() command
    waitKey(30);
  }

  return 0;
}
