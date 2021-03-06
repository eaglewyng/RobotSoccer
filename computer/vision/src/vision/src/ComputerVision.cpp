//============================================================================
// Name : ComputerVision.cpp
// Author : Luke Hsiao, Clover Wu
// Version :
// Copyright : Copyright 2015 Team Vektor Krum
// Description : This program receives video data from the Soccer Field's
// overhead camera, processes the images, and outputs
// the (x,y) positions of all 4 robots, the ball, and all
// robot's orientations.
//============================================================================

#include "ComputerVision.h"
#include "Ball.h"
#include "Robot.h"
#include "Object.h"


using namespace cv;

// Change this parameter to determine which team we are on!
// Either set it to HOME or AWAY
int TEAM = HOME;

// Change this parameter to switch to performance, no GUI mode
// Either set it to GUI or NO_GUI
int PERF_MODE = GUI;

// Field variables
int field_width;
int field_height;
int field_center_x;
int field_center_y;
int home_field_width;
int home_field_height;
int home_field_center_x;
int home_field_center_y;
int away_field_width;
int away_field_height;
int away_field_center_x;
int away_field_center_y;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

//min and max field variable values
int field_height_min = 0;
int field_width_min = 0;
int field_center_x_min = 0;
int field_center_y_min = 0;
int field_height_max = FRAME_HEIGHT + 300;
int field_width_max = FRAME_WIDTH + 300;
int field_center_x_max = FRAME_WIDTH;
int field_center_y_max = FRAME_HEIGHT;

//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS=50;

//minimum and maximum object area
const int MIN_ROBOT_AREA = 10*10;
const int MIN_BALL_AREA = 5*5;
const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

//names that will appear at the top of each window
const string windowName = "Original Image";
const string windowName1 = "HSV Image";
const string windowName2 = "Thresholded Image";
const string windowName3 = "After Morphological Operations";
const string trackbarWindowName = "Trackbars";

// Camera Calibration Data
double dist_coeff[5][1] = {  {-0.3647790},
                             {0.184763},
                             {0.003989429},
                             {-0.0003392181},
                             {-0.06141144}
                           };

//double dist_coeff[5][1] = {  {-0.43},
//                             {0.23},
//                             {-0.004},
//                             {-0.005},
//                             {-0.07}
//                           };


//double cam_matrix[3][3] = {  {846,  0.0,  639.5},
//                             {0.0,  849,  436.0},
//                             {0.0,  0.0,  1.0}
//                          };


double cam_matrix[3][3] = {  {513,  0.0,  315.6},
                             {0.0,  513.94,  266.72},
                             {0.0,  0.0,  1.0}
                          };

// Declaration of the 5 objects
Robot home1(HOME), home2(HOME);
Robot away1(AWAY), away2(AWAY);
Ball ball;

// threading
sem_t frameRawSema;
std::queue<FrameMat> frameRawFifo;
sem_t frameMatSema;
std::queue<FrameMat> frameMatFifo;
Mat cameraFeed;
Mat HSV;
sem_t trackBallBegin;
sem_t trackHome1Begin;
sem_t trackHome2Begin;
sem_t trackAway1Begin;
sem_t trackAway2Begin;
sem_t trackBallEnd;
sem_t trackHome1End;
sem_t trackHome2End;
sem_t trackAway1End;
sem_t trackAway2End;
sem_t cameraChangeSem;

string videoStreamAddress;
VideoCapture capture;

bool filterInitialized;
robot_soccer::locations oldLocations;

ros::NodeHandle* nodeHandle;

//if we would like to calibrate our filter values, set to true.
bool calibrationMode = true;

void restoreFieldValues() {
  field_width = (TEAM == HOME) ? home_field_width : away_field_width;
  field_height = (TEAM == HOME) ? home_field_height : away_field_height;
  field_center_x = (TEAM == HOME) ? home_field_center_x : away_field_center_x;
  field_center_y = (TEAM == HOME) ? home_field_center_y : away_field_center_y;
}

// Saves all of the pertinent calibration settings to human-readable file
void saveSettings() {
  // Open file
  std::ofstream of("settings.data");

  if (!of.is_open()) {
    printf("\n\n\nERROR OPENING SETTINGS FILE!\n\n\n");
    return;
  }

  const string h1 = "Home1";
  const string h2 = "Home2";
  const string a1 = "Away1";
  const string a2 = "Away2";
  const string b = "Ball";
  const string hf = "HomeField";
  const string af = "AwayField";
  int tempH, tempS, tempV;

  // Print human-readable header for settings file
  of << "#############################################################" << "\n";
  of << "# This settings file contains the color calibration settings" << "\n";
  of << "# Format: " << "\n";
  of << "# [RobotName] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]" << "\n";
  of << "# [Ball] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]" << "\n";
  of << "# [Field] [x_center_pos] [y_center_pos] [width] [height]" << "\n";
  of << "#############################################################" << "\n";
  of << "\n\n";

  // Robot Save format:
  // [RobotName] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]
  tempH = home1.getHSVmin().val[0];
  tempS = home1.getHSVmin().val[1];
  tempV = home1.getHSVmin().val[2];
  of << h1 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = home1.getHSVmax().val[0];
  tempS = home1.getHSVmax().val[1];
  tempV = home1.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = home2.getHSVmin().val[0];
  tempS = home2.getHSVmin().val[1];
  tempV = home2.getHSVmin().val[2];
  of << h2 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = home2.getHSVmax().val[0];
  tempS = home2.getHSVmax().val[1];
  tempV = home2.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = away1.getHSVmin().val[0];
  tempS = away1.getHSVmin().val[1];
  tempV = away1.getHSVmin().val[2];
  of << a1 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = away1.getHSVmax().val[0];
  tempS = away1.getHSVmax().val[1];
  tempV = away1.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  tempH = away2.getHSVmin().val[0];
  tempS = away2.getHSVmin().val[1];
  tempV = away2.getHSVmin().val[2];
  of << a2 << " " <<  tempH << " " <<  tempS << " " <<  tempV;
  tempH = away2.getHSVmax().val[0];
  tempS = away2.getHSVmax().val[1];
  tempV = away2.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";


  // Ball Save format:
  // [Ball] [Hmin] [Smin] [Vmin] [Hmax] [Smax] [Vmax]
  tempH = ball.getHSVmin().val[0];
  tempS = ball.getHSVmin().val[1];
  tempV = ball.getHSVmin().val[2];
  of << b << " " <<  tempH << " " << tempS << " " <<  tempV;
  tempH = ball.getHSVmax().val[0];
  tempS = ball.getHSVmax().val[1];
  tempV = ball.getHSVmax().val[2];
  of << " " <<  tempH << " " <<  tempS << " " <<  tempV << "\n";

  // Field Save format:
  // [Field] [x_pos] [y_pos] [width] [height]
  if (TEAM == HOME) {
    of << hf << " " << field_center_x << " " << field_center_y;
    of << " " << field_width << " " <<  field_height << "\n";

    of << af << " " << away_field_center_x << " " << away_field_center_y;
    of << " " << away_field_width << " " <<  away_field_height << "\n";
  }
  else {
    of << hf << " " << home_field_center_x << " " << home_field_center_y;
    of << " " << home_field_width << " " <<  home_field_height << "\n";

    of << af << " " << field_center_x << " " << field_center_y;
    of << " " << field_width << " " <<  field_height << "\n";
  }

  // close file
  of.close();
  printf("Settings Saved!\n");
}

// Reads in saved settings from settings.data and stores them in objects
void restoreSettings() {
  const string h1 = "Home1";
  const string h2 = "Home2";
  const string a1 = "Away1";
  const string a2 = "Away2";
  const string b = "Ball";
  const string hf = "HomeField";
  const string af = "AwayField";
  int tempH, tempS, tempV;

  std::stringstream ss;
  std::ifstream in("settings.data");
  if (!in.good()) {
    printf("\n\n\nERROR, Couldn't open file\n\n\n");
    return;
  }

  // char line[MAX_CHARS];
  string ID;
  string line;

  // Parse File line by line
  while(getline(in, line)) {
    if (line.c_str()[0] == '#') {
      continue; //skip comments
    }
    ss.str(line);
    ss >> ID;
    if (ID == h1) {
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      home1.setHSVmin(Scalar(tempH, tempS, tempV));
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      home1.setHSVmax(Scalar(tempH, tempS, tempV));
    }
    else if (ID == h2) {
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      home2.setHSVmin(Scalar(tempH, tempS, tempV));
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      home2.setHSVmax(Scalar(tempH, tempS, tempV));
    }
    else if (ID == a1) {
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      away1.setHSVmin(Scalar(tempH, tempS, tempV));
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      away1.setHSVmax(Scalar(tempH, tempS, tempV));
    }
    else if (ID == a2) {
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      away2.setHSVmin(Scalar(tempH, tempS, tempV));
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      away2.setHSVmax(Scalar(tempH, tempS, tempV));
    }
    else if (ID == b) {
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      ball.setHSVmin(Scalar(tempH, tempS, tempV));
      ss >> tempH;
      ss >> tempS;
      ss >> tempV;
      ball.setHSVmax(Scalar(tempH, tempS, tempV));
    }
    else if (ID == hf) {
      ss >> home_field_center_x;
      ss >> home_field_center_y;
      ss >> home_field_width;
      ss >> home_field_height;
    }
    else if (ID == af) {
      ss >> away_field_center_x;
      ss >> away_field_center_y;
      ss >> away_field_width;
      ss >> away_field_height;
    }
    ss.clear();
  }
  in.close();
  printf("Settings Restored!\n");
}

//-----------------------------------------------------------------------------
// Utility Functions Shared by Classes
//-----------------------------------------------------------------------------

// This function is called whenever a trackbar changes
void on_trackbar( int, void* ) {
  // Does nothing
}

string intToString(int number) {
  std::stringstream ss;
  ss << number;
  return ss.str();
}

// Runs the undistortion
void undistortImage(Mat &source) {
  return;
  // Setup Distortion matrices
  Mat cameraMatrix = Mat(3, 3, CV_64F, cam_matrix); // read in 64-bit doubles
  Mat distCoeffs = Mat(5, 1, CV_64F, dist_coeff);

  double y_shift = 60;
  double x_shift = 70;
  int enlargement = 185;
  Size imageSize = source.size();
  imageSize.height += enlargement;
  imageSize.width += enlargement;
  Mat temp = source.clone();

  Mat newCameraMatrix = cameraMatrix.clone();

  // Adjust the position of the newCameraMatrix
  // at() is a templated function so <double> is necessary
  newCameraMatrix.at<double>(1,2) += y_shift; //shift the image down
  newCameraMatrix.at<double>(0,2) += x_shift; //shift the image right

  Mat map1;
  Mat map2;
  initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix, imageSize, CV_16SC2, map1, map2);

  remap(temp, source, map1, map2, INTER_LINEAR);
}

void createHSVTrackbars() {
	//create window for trackbars
	namedWindow(trackbarWindowName,0);

	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH),
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );
}

//Converts from image coordinates to field coordinates
Point convertCoordinates(Point imageCoordinates) {
  int img_x = imageCoordinates.x;
  int img_y = imageCoordinates.y;

  int field_x;
  int field_y;
  Point result;

  field_x = img_x - (field_width/2);

  field_y = (field_height/2) - img_y;

  result.x = field_x;
  result.y = field_y;
  return result;
}

// This function reduces the noise of the image by eroding the image first
// then dialating the remaining image to produce cleaner objects
void morphOps(Mat &thresh) {
  //create structuring element that will be used to "dilate" and "erode" image.

  //the element chosen here is a 3px by 3px rectangle
  Mat erodeElement = getStructuringElement( MORPH_RECT,Size(2,2));
  //dilate with larger element so make sure object is nicely visible
  Mat dilateElement = getStructuringElement( MORPH_RECT,Size(3,3));

  erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);

  erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
  //dilate with larger element so make sure object is nicely visible
  dilateElement = getStructuringElement( MORPH_RECT,Size(6,6));

  erode(thresh,thresh,erodeElement);
  dilate(thresh,thresh,dilateElement);
}
//-----------------------------------------------------------------------------
//  End of Utility Functions Shared by Classes
//-----------------------------------------------------------------------------

// Generates prompts for field calibration of size/center
void calibrateField() {
  restoreSettings();
  restoreFieldValues();
  VideoCapture calibrationCapture(videoStreamAddress);
  Mat cameraFeed;
  int field_origin_x;
  int field_origin_y;

  //create window for trackbars
  namedWindow(trackbarWindowName,0);

  //create trackbars and insert them into window
  //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
  //the max value the trackbar can move (eg. H_HIGH),
  //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
  createTrackbar( "Field Center Y", trackbarWindowName, &field_center_y_min, field_center_y_max, on_trackbar );
  createTrackbar( "Field Center X", trackbarWindowName, &field_center_x_min, field_center_x_max, on_trackbar );
  createTrackbar( "Field Height", trackbarWindowName, &field_height_min, field_height_max, on_trackbar );
  createTrackbar( "Field Width", trackbarWindowName, &field_width_min, field_width_max, on_trackbar );

  // Set Trackbar Initial Positions
  setTrackbarPos( "Field Center Y", trackbarWindowName, field_center_y);
  setTrackbarPos( "Field Center X", trackbarWindowName, field_center_x);
  setTrackbarPos( "Field Height", trackbarWindowName, field_height);
  setTrackbarPos( "Field Width", trackbarWindowName, field_width);

  // Wait forever until user sets the values
  while (1) {
    calibrationCapture.read(cameraFeed);
    undistortImage(cameraFeed);

    // Wait for user to set values
    field_center_y = getTrackbarPos( "Field Center Y", trackbarWindowName);
    field_center_x = getTrackbarPos( "Field Center X", trackbarWindowName);
    field_height = getTrackbarPos( "Field Height", trackbarWindowName);
    field_width = getTrackbarPos( "Field Width", trackbarWindowName);

    field_origin_x = field_center_x - (field_width/2);
    field_origin_y = field_center_y - (field_height/2);

    Rect fieldOutline(field_origin_x, field_origin_y, field_width, field_height);

    // Draw centering lines
    Point top_mid(field_center_x, field_origin_y);
    Point bot_mid(field_center_x, field_origin_y+field_height);
    Point left_mid(field_origin_x, field_center_y);
    Point right_mid(field_origin_x+field_width, field_center_y);
    line(cameraFeed,top_mid, bot_mid, Scalar(200,200,200), 1, 8, 0);
    line(cameraFeed,left_mid, right_mid, Scalar(200,200,200), 1, 8, 0);

    // Draw outline
    rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);
    imshow(windowName,cameraFeed);

    char pressedKey;
    pressedKey = cvWaitKey(1); // Wait for user to press 'Enter'
    if (pressedKey == '\n') {
      field_center_y = getTrackbarPos( "Field Center Y", trackbarWindowName);
      field_center_x = getTrackbarPos( "Field Center X", trackbarWindowName);
      field_height = getTrackbarPos( "Field Height", trackbarWindowName);
      field_width = getTrackbarPos( "Field Width", trackbarWindowName);

      printf("\n\nField Values Saved!\n");
      printf("Field Center Y: %d\n", field_center_y);
      printf("Field Center X: %d\n", field_center_x);
      printf("Field Width: %d\n", field_width);
      printf("Field Height: %d\n", field_height);
      destroyAllWindows();
      calibrationCapture.release();
      saveSettings();
      return;
    }
  }
}

// Generates all the calibration prompts (field + ball + robots)
void calibrateObjects() {
  restoreSettings();
  ball.calibrateBall(capture);
  home1.calibrateRobot(capture);
  home2.calibrateRobot(capture);
  away1.calibrateRobot(capture);
  away2.calibrateRobot(capture);
  saveSettings();
}

// Special function for both getting the next image and reading the timestamp
// from the IP camera. Currently NOT very optimized for performance.
ros::Time getNextImage(std::ifstream & myFile, std::vector<char> & imageArray) {
  imageArray.clear();
  imageArray.reserve(1024*64);
  char buffer[4];
  ros::Time timestamp;
  bool foundImage = false;
  while(!foundImage){
    myFile.read(buffer,1);
    if((*buffer) == (char)0xFF){
      myFile.read(buffer,1);
      if((*buffer) == (char)0xD8){
        //printf("found start of image \n");
        imageArray.push_back((char)0xFF);
        imageArray.push_back((char)0xD8);
        while(1){
          myFile.read(buffer,1);
          imageArray.push_back(*buffer);
          if((*buffer) == (char)0xFF){
            myFile.read(buffer,1);
            imageArray.push_back(*buffer);
            if((*buffer) == (char)0xFE){
              myFile.read(buffer,4);
              imageArray.push_back(*buffer);
              imageArray.push_back(*(buffer+1));
              imageArray.push_back(*(buffer+2));
              imageArray.push_back(*(buffer+3));
              if((*(buffer+3)) == (char)0x01) {
                myFile.read(buffer,4);
                unsigned int sec = 0;
                for(int i = 0; i < 4; i++){
                  imageArray.push_back(*(buffer + i));
                  sec <<= 8;
                  sec += *(unsigned char*)(void*)(buffer+i);
                }

                myFile.read(buffer,1);
                unsigned int hundreds = 0;
                imageArray.push_back(*buffer);
                hundreds += *(unsigned char*)(void*)buffer;
                timestamp.sec = sec;
                timestamp.nsec = hundreds * 10000000;
              }


            }else if((*buffer) == (char)0xD9){
              //printf("found end of image\n");
              foundImage = true;
              break;
            }
          }
        }
      }
    }
  }
  return timestamp;
}

//This thread loads the streaming video into memory to be loaded into
//openCV
void * parserThread(void * notUsed){
  FrameMat frame;

  do {
    sem_wait(&cameraChangeSem);
    ros::Time timestamp;
    timestamp.sec = 0;
    timestamp.nsec = 0;
    frame.timestamp = timestamp;
    capture.read(frame.image);
    // flip(frame.image, frame.image, -1);
    sem_post(&cameraChangeSem);

    int value;
    sem_getvalue(&frameRawSema, &value);
    if (value < MIN_BUFFER_SIZE){
      frameRawFifo.push(frame);
      sem_post(&frameRawSema);
    }
    else {
      //printf("frame dropped (Capture): %u.%09u\n",frame.timestamp.sec,frame.timestamp.nsec);
    }
  } while (1);
  return NULL;
}

//This thread converts JPEGs into Mats and undistorts them.
void * processorThread(void * notUsed){
  FrameMat frameRaw;
  FrameMat frameMat;
  while(1) {
    sem_wait(&frameRawSema);
    frameRaw = frameRawFifo.front();
    frameRawFifo.pop();

    int value;
    sem_getvalue(&frameMatSema, &value);
    if (value < MIN_BUFFER_SIZE){
      frameMat.timestamp = frameRaw.timestamp;
      frameMat.image = frameRaw.image;

      undistortImage(frameMat.image);

      int field_origin_x;
      int field_origin_y;

      //convert frame from BGR to HSV colorspace
      field_origin_x = field_center_x - (field_width/2);
      field_origin_y = field_center_y - (field_height/2);
      Rect myROI(field_origin_x,field_origin_y,field_width, field_height);
      frameMat.image = frameMat.image(myROI);

      cvtColor(frameMat.image,frameMat.HSV,COLOR_BGR2HSV);

      frameMatFifo.push(frameMat);
      sem_post(&frameMatSema);
    }
    else {
      //printf("frame dropped (Process): %u.%09u\n",frameRaw.timestamp.sec,frameRaw.timestamp.nsec);
    }
  }
  return NULL;
}

// Track Ball
void * ballThread(void * notUsed) {
    Mat threshold;
    while(1) {
      sem_wait(&trackBallBegin);
      inRange(HSV,ball.getHSVmin(),ball.getHSVmax(),threshold);
      ball.trackFilteredBall(threshold,HSV,cameraFeed);
      //printf("ball tracked");
      sem_post(&trackBallEnd);
    }
}

// Track home1
void * home1Thread(void * notUsed) {
    Mat threshold;
    while(1) {
      sem_wait(&trackHome1Begin);
      inRange(HSV,home1.getHSVmin(),home1.getHSVmax(),threshold);
      home1.trackFilteredRobot(threshold,HSV,cameraFeed);
      //printf("home1 tracked");
      sem_post(&trackHome1End);
    }
}

// Track home2
void * home2Thread(void * notUsed) {
    Mat threshold;
    while(1) {
      sem_wait(&trackHome2Begin);
      inRange(HSV,home2.getHSVmin(),home2.getHSVmax(),threshold);
      home2.trackFilteredRobot(threshold,HSV,cameraFeed);
      //printf("home2 tracked");
      sem_post(&trackHome2End);
    }
}

// Track away1
void * away1Thread(void * notUsed) {
    Mat threshold;
    while(1) {
      sem_wait(&trackAway1Begin);
      inRange(HSV,away1.getHSVmin(),away1.getHSVmax(),threshold);
      away1.trackFilteredRobot(threshold,HSV,cameraFeed);
      //printf("Away1 tracked");
      sem_post(&trackAway1End);
    }
}

// Track away2
void * away2Thread(void * notUsed) {
    Mat threshold;
    while(1) {
      sem_wait(&trackAway2Begin);
      inRange(HSV,away2.getHSVmin(),away2.getHSVmax(),threshold);
      away2.trackFilteredRobot(threshold,HSV,cameraFeed);
      //printf("Away2 tracked");
      sem_post(&trackAway2End);
    }
}

void changeCamera(const std_msgs::Int32 message) {
  sem_wait(&cameraChangeSem);
  if (capture.isOpened()) {
    capture.release();
  }

  TEAM = message.data;
  std::string ip = (TEAM == HOME) ? "192.168.1.78" : "192.168.1.79";
  videoStreamAddress = "http://" + ip + ":8080/stream?topic=/image&dummy=param.mjpg";
  capture = VideoCapture(videoStreamAddress);

  //set height and width of capture frame
  capture.set(CV_CAP_PROP_FRAME_WIDTH,FRAME_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT,FRAME_HEIGHT);
  sem_post(&cameraChangeSem);

  if (calibrationMode == true) {
    calibrateField();
  }
  restoreSettings();
  restoreFieldValues();
}

bool thetaChange(int newTheta, int oldTheta) {
  int diff = newTheta - oldTheta;
  if (abs(diff) > 180) {
    if (diff > 0) {
      diff = diff - 360;
    }
    else {
      diff = diff + 360;
    }
  }
  return diff;
}

robot_soccer::locations lowPassFilterLocations(robot_soccer::locations measuredLocations) {
  robot_soccer::locations processedLocations;

  // initialize filter
  if (!filterInitialized) {
    filterInitialized = true;
    oldLocations.ball_x = 0;
    oldLocations.ball_y = 0;
    oldLocations.home1_x = 0;
    oldLocations.home1_y = 0;
    oldLocations.home1_theta = 0;
    oldLocations.home2_x = 0;
    oldLocations.home2_y = 0;
    oldLocations.home2_theta = 0;
    oldLocations.away1_x = 0;
    oldLocations.away1_y = 0;
    oldLocations.away1_theta = 0;
    oldLocations.away2_x = 0;
    oldLocations.away2_y = 0;
    oldLocations.away2_theta = 0;
  }

  double cutoff_frequency = 0.0;//360.0; // roughly 1 rev/s
  double Ts = 1.0/40.0; // sample at 40 times a second
  double alpha = 2*3.14*Ts*cutoff_frequency/(2*3.14*Ts*cutoff_frequency + 1);

  // filter measured values
  processedLocations.ball_x      = alpha*oldLocations.ball_x      + (1-alpha)*measuredLocations.ball_x;
  processedLocations.ball_y      = alpha*oldLocations.ball_y      + (1-alpha)*measuredLocations.ball_y;
  processedLocations.home1_x     = alpha*oldLocations.home1_x     + (1-alpha)*measuredLocations.home1_x;
  processedLocations.home1_y     = alpha*oldLocations.home1_y     + (1-alpha)*measuredLocations.home1_y;
  processedLocations.home1_theta = alpha*oldLocations.home1_theta + (1-alpha)*measuredLocations.home1_theta;
  processedLocations.home2_x     = alpha*oldLocations.home2_x     + (1-alpha)*measuredLocations.home2_x;
  processedLocations.home2_y     = alpha*oldLocations.home2_y     + (1-alpha)*measuredLocations.home2_y;
  processedLocations.home2_theta = alpha*oldLocations.home2_theta + (1-alpha)*measuredLocations.home2_theta;
  processedLocations.away1_x     = alpha*oldLocations.away1_x     + (1-alpha)*measuredLocations.away1_x;
  processedLocations.away1_y     = alpha*oldLocations.away1_y     + (1-alpha)*measuredLocations.away1_y;
  processedLocations.away1_theta = alpha*oldLocations.away1_theta + (1-alpha)*measuredLocations.away1_theta;
  processedLocations.away2_x     = alpha*oldLocations.away2_x     + (1-alpha)*measuredLocations.away2_x;
  processedLocations.away2_y     = alpha*oldLocations.away2_y     + (1-alpha)*measuredLocations.away2_y;
  processedLocations.away2_theta = alpha*oldLocations.away2_theta + (1-alpha)*measuredLocations.away2_theta;


  if (thetaChange(measuredLocations.home1_theta, oldLocations.home1_theta) > 50) {
    processedLocations.home1_theta = oldLocations.home1_theta;
  }
  if (thetaChange(measuredLocations.home2_theta, oldLocations.home2_theta) > 50) {
    processedLocations.home2_theta = oldLocations.home2_theta;
  }
  if (thetaChange(measuredLocations.away1_theta, oldLocations.away1_theta) > 50) {
    processedLocations.away1_theta = oldLocations.away1_theta;
  }
  if (thetaChange(measuredLocations.away2_theta, oldLocations.away2_theta) > 50) {
    processedLocations.away2_theta = oldLocations.away2_theta;
  }

  processedLocations.header.stamp = measuredLocations.header.stamp;

  // printf("old = %d, measured = %d, processed = %d\n", oldLocations.home1_theta, measuredLocations.home1_theta, processedLocations.home1_theta);

  // update old values
  oldLocations.ball_x      = processedLocations.ball_x;
  oldLocations.ball_y      = processedLocations.ball_y;
  oldLocations.home1_x     = processedLocations.home1_x;
  oldLocations.home1_y     = processedLocations.home1_y;
  oldLocations.home1_theta = processedLocations.home1_theta;
  oldLocations.home2_x     = processedLocations.home2_x;
  oldLocations.home2_y     = processedLocations.home2_y;
  oldLocations.home2_theta = processedLocations.home2_theta;
  oldLocations.away1_x     = processedLocations.away1_x;
  oldLocations.away1_y     = processedLocations.away1_y;
  oldLocations.away1_theta = processedLocations.away1_theta;
  oldLocations.away2_x     = processedLocations.away2_x;
  oldLocations.away2_y     = processedLocations.away2_y;
  oldLocations.away2_theta = processedLocations.away2_theta;

  return processedLocations;
}

int main(int argc, char* argv[]) {
	//Matrix to store each frame of the webcam feed
	Mat threshold1; //threshold image of ball
	Mat threshold2; //threshold image of robot
	Mat threshold; //combined image
	Mat bw; // black and white mat
  Mat BGR;// BGR mat

  //namedWindow(windowName,WINDOW_NORMAL);

  /***********************Ros************************************/

  ros::init(argc, argv, "computer_vision");
  nodeHandle = new ros::NodeHandle();
  ros::Publisher publisher = nodeHandle->advertise<robot_soccer::locations>("locTopic", 1000);
  ros::Subscriber cameraSub = nodeHandle->subscribe("visionCamera", 10, changeCamera);
  ros::Rate loop_rate(40);

  // restore settings
  restoreSettings();
  restoreFieldValues();

  // initialize camera to home
  sem_init(&cameraChangeSem, 0, 1);
  std_msgs::Int32 message;
  message.data = HOME;
  changeCamera(message);

  // calibrate robots
  if (calibrationMode) {
    calibrateObjects();
  }

  /************************************************************************/


  /************************************************************************/
  //start an infinite loop where webcam feed is copied to cameraFeed matrix
  //all of our operations will be performed within this loop

  pthread_t parser;
  pthread_t processor;
  pthread_t ballT;
  pthread_t away1T;
  pthread_t away2T;
  pthread_t home1T;
  pthread_t home2T;
  sem_init(&frameRawSema,0,0);
  sem_init(&frameMatSema,0,0);
  sem_init(&trackBallBegin,0,0);
  sem_init(&trackHome1Begin,0,0);
  sem_init(&trackHome2Begin,0,0);
  sem_init(&trackAway1Begin,0,0);
  sem_init(&trackAway2Begin,0,0);
  sem_init(&trackBallEnd,0,0);
  sem_init(&trackHome1End,0,0);
  sem_init(&trackHome2End,0,0);
  sem_init(&trackAway1End,0,0);
  sem_init(&trackAway2End,0,0);
  pthread_create (&parser, NULL, parserThread, NULL);
  pthread_create (&processor, NULL, processorThread, NULL);
  pthread_create (&ballT, NULL, ballThread, NULL);
  pthread_create (&home1T, NULL, home1Thread, NULL);
  pthread_create (&home2T, NULL, home2Thread, NULL);
  pthread_create (&away1T, NULL, away1Thread, NULL);
  pthread_create (&away2T, NULL, away2Thread, NULL);

  filterInitialized = false;

  unsigned int printCounter = 0;
  while(ros::ok()) {
    ros::spinOnce();

    sem_wait(&frameMatSema);
    FrameMat frame = frameMatFifo.front();
    frameMatFifo.pop();

    //store image to matrix
    //capture.read(cameraFeed);
	  ros::Time timestamp = frame.timestamp;
	  //printf("%u.%09u\n",timestamp.sec,timestamp.nsec);
    cameraFeed = frame.image;
    HSV = frame.HSV;

    sem_post(&trackBallBegin);
    sem_post(&trackHome1Begin);
    sem_post(&trackHome2Begin);
    sem_post(&trackAway1Begin);
    sem_post(&trackAway2Begin);
    sem_wait(&trackBallEnd);
    sem_wait(&trackHome1End);
    sem_wait(&trackHome2End);
    sem_wait(&trackAway1End);
    sem_wait(&trackAway2End);

    if (PERF_MODE == GUI /* && !(printCounter%PRINT_FREQ) */) {
      // Show Field Outline
      Rect fieldOutline(0, 0, field_width, field_height);
      rectangle(cameraFeed,fieldOutline,Scalar(255,255,255), 1, 8 ,0);

      // Draw centering lines
      Point top_mid(field_width/2, 0);
      Point bot_mid(field_width/2, field_height);
      Point left_mid(0, field_height/2);
      Point right_mid(field_width, field_height/2);
      line(cameraFeed,top_mid, bot_mid, Scalar(200,200,200), 1, 8, 0);
      line(cameraFeed,left_mid, right_mid, Scalar(200,200,200), 1, 8, 0);

      //create window for trackbars
      imshow(windowName,cameraFeed);
    }

    /***********************Ros Publisher************************************/

    // Create message object
    robot_soccer::locations coordinates;
    // Fill message object with values
    coordinates.ball_x = ball.get_x_pos();
    coordinates.ball_y = ball.get_y_pos();
    coordinates.home1_x = home1.get_x_pos();
    coordinates.home1_y = home1.get_y_pos();
    coordinates.home1_theta = home1.getAngle();
    coordinates.home2_x = home2.get_x_pos();
    coordinates.home2_y = home2.get_y_pos();
    coordinates.home2_theta = home2.getAngle();
    coordinates.away1_x = away1.get_x_pos();
    coordinates.away1_y = away1.get_y_pos();
    coordinates.away1_theta = away1.getAngle();
    coordinates.away2_x = away2.get_x_pos();
    coordinates.away2_y = away2.get_y_pos();
    coordinates.away2_theta = away2.getAngle();
    coordinates.header.stamp = timestamp;

    coordinates = lowPassFilterLocations(coordinates);

    // Print values to ROS console
    if (false && !(printCounter%PRINT_FREQ)) {
      ROS_INFO("\n  timestamp: %u.%09u\n  ", coordinates.header.stamp.sec, coordinates.header.stamp.nsec);
      //ROS_INFO("\n  Ball_x: %d\n  Ball_y: %d\n", coordinates.ball_x, coordinates.ball_y);
    }
    // Publish message
    publisher.publish(coordinates);
    // Waits the necessary time between message publications to meet the
    // specified frequency set above (ros::Rate loop_rate(10);)
    loop_rate.sleep();

    /************************************************************************/



		//image will not appear without this waitKey() command
		// Wait to check if user wants to switch Home/Away
    char pressedKey;
    pressedKey = cvWaitKey(1); // Wait for user to press 'Enter'
    if (pressedKey == 'a') {
      TEAM = AWAY;
    }
    else if (pressedKey == 'h') {
      TEAM = HOME;
    }
		printCounter++;
	}
	return 0;
}

