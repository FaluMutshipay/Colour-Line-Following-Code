// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
#include "main.hpp"     // You can use this file for declaring defined values and functions
#include "pi2c.h"
#include <iostream>
#include <math.h>
//#include <Wire.h>
//#include <opencv2core.hpp>
//#include <opencv2highgui.hpp>
//#include <opencv2imgproc.hpp>

using namespace cv;
using namespace std;

Pi2c car(0x22); // Configure the I2C interface to the Car as a global variable

void setup(void)
{
    setupCamera(320, 240);  // Enable the camera for OpenCV
}

int main( int argc, char** argv )
{
    //Initialise some variables for HSV limits
    int lowH_red = 0, highH_red = 9, lowS_red = 50, lowV_red = 150;
    int lowH_orange  = 10, highH_orange = 24, lowS_orange = 50, lowV_orange = 70;
    int lowH_yellow = 25, highH_yellow = 35, lowS_yellow = 50, lowV_yellow = 70;
    int lowH_green = 36, highH_green = 89, lowS_green = 50, lowV_green = 100;
    int lowH_blue = 90, highH_blue = 128, lowS_blue = 50, lowV_blue = 70;
    int lowH_purple = 129, highH_purple = 158, lowS_purple = 50, lowV_purple = 70;
    int lowH_mono = 0,highH_mono = 180, lowS_mono = 0, highS_mono = 255, lowV_black = 0, highV_black = 15;
    int lowV_white = 230, highV_white = 255;
    int highS = 255, highV = 255;
    int redCount, purpleCount, blueCount, greenCount, orangeCount, yellowCount, whiteCount, blackCount;

    int max_sensorColumn=320;
    int max_sensorRow=240;

    float weightedAverage;
    float Kp = 1;
    float Ki = 0;
    float Kd = 0.5;
    int16_t leftMotor_speed = 0;
    int16_t rightMotor_speed = 0;
    int leftSpeed_setting=150;
    int rightSpeed_setting=150;
    int centreAngle=93;
    int servoAngle =0;
    Point regionCentre;



    setup();    // Call a setup function to prepare IO and devices


    cv::namedWindow("Photo");   // Create a GUI window called photo

    while(1)    // Main loop to perform image processing
    {
        Mat frame;

        while(frame.empty())
            frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
            flip(frame,frame,0);


        cv::imshow("Photo", frame); //Display the image in the window

    // Convert the frame to HSV and apply the limits
    Mat frameHSV, frameHSV_red, frameHSV_purple, frameHSV_orange, frameHSV_yellow, frameHSV_blue, frameHSV_green, frameHSV_black, frameHSV_white;
    cvtColor(frame, frameHSV, COLOR_BGR2HSV);

    //  GET RED COLOUR
    inRange(frameHSV, Scalar(lowH_red, lowS_red, lowV_red), Scalar(highH_red, highS, highV), frameHSV_red);

     //  GET ORANGE COLOUR
    inRange(frameHSV, Scalar(lowH_orange, lowS_orange, lowV_red), Scalar(highH_orange, highS, highV), frameHSV_orange);

    // GET YELLOW COLOUR
    inRange(frameHSV, Scalar(lowH_yellow, lowS_yellow, lowV_yellow), Scalar(highH_yellow, highS, highV), frameHSV_yellow);

    // GET BLUE COLOUR
    inRange(frameHSV, Scalar(lowH_blue, lowS_blue, lowV_blue), Scalar(highH_blue, highS, highV), frameHSV_blue);

    // GET GREEN COLOUR
    inRange(frameHSV, Scalar(lowH_green, lowS_green, lowV_green), Scalar(highH_green, highS, highV), frameHSV_green);

    // GET BLACK COLOUR
    inRange(frameHSV, Scalar(lowH_mono, lowS_mono, lowV_black), Scalar(highH_mono, highS_mono, highV_black), frameHSV_black);

    // GET WHITE COLOUR
    inRange(frameHSV, Scalar(lowH_mono, lowS_mono, lowV_white), Scalar(highH_mono, highS_mono, highV_white), frameHSV_white);

    // GET PURPLE COLOUR
    inRange(frameHSV, Scalar(lowH_purple, lowS_purple, lowV_purple), Scalar(highH_purple, highS, highV), frameHSV_purple);

    redCount = countNonZero(frameHSV_red);
    orangeCount = countNonZero(frameHSV_orange);
    yellowCount = countNonZero(frameHSV_yellow);
    blueCount = countNonZero(frameHSV_blue);
    greenCount = countNonZero(frameHSV_green);
    blackCount = countNonZero(frameHSV_black);
    whiteCount = countNonZero(frameHSV_white);
    purpleCount = countNonZero(frameHSV_purple);

    if ((redCount>blueCount)&&(redCount>greenCount)&&(redCount>yellowCount)&&(redCount>orangeCount)&&(redCount>blackCount)&&(redCount>whiteCount)&&(redCount>purpleCount)){
       cout<<"MAIN OBJECT IS RED"<<endl;
       cout<<redCount<<endl;
       hconcat(frame, frameHSV, frameHSV_red);
       //imshow("Red Output", frameHSV_red);
    }
    /* else if (((orangeCount>redCount1)&&(orangeCount>greenCount)&&(orangeCount>yellowCount)&&(orangeCount>blueCount)&&(orangeCount>blackCount)&&(orangeCount>whiteCount))||((orangeCount>redCount2)&&(orangeCount>greenCount)&&(orangeCount>yellowCount)&&(orangeCount>blueCount)&(orangeCount>blackCount)&&(orangeCount>whiteCount))){

            cout<<"MAIN OBJECT IS ORANGE"<<endl;
            cout<<orangeCount<<endl;
            hconcat(frame, frameHSV, frameHSV_orange);
            //imshow("GOrange Output", frameHSV_orange);
    }
     else if (((yellowCount>redCount)&&(yellowCount>greenCount)&&(yellowCount>orangeCount)&&(yellowCount>blueCount)&&(yellowCount>blackCount)&&(yellowCount>whiteCount))||((yellowCount>redCount2)&&(yellowCount>greenCount)&&(yellowCount>orangeCount)&&(yellowCount>blueCount)&&(yellowCount>blackCount)&&(yellowCount>whiteCount))){

            cout<<"MAIN OBJECT IS YELLOW"<<endl;
            cout<<yellowCount<<endl;
            hconcat(frame, frameHSV, frameHSV_yellow);
            //imshow("Yellow Output", frameHSV_yellow);
    }*/
    /*else if ((blueCount>redCount)&&(blueCount>greenCount)&&(blueCount>orangeCount)&&(blueCount>yellowCount)&&(blueCount>blackCount)&&(blueCount>whiteCount)&&(blueCount>purpleCount)){

            cout<<"MAIN OBJECT IS BLUE"<<endl;
            cout<<blueCount<<endl;
            hconcat(frame, frameHSV, frameHSV_blue);
            //imshow("Blue Output", frameHSV_blue);
    }*/
    else if ((greenCount>redCount)&&(greenCount>blueCount)&&(greenCount>orangeCount)&&(greenCount>yellowCount)&&(greenCount>blackCount)&&(greenCount>whiteCount)&&(greenCount>purpleCount)){
        cout<<"MAIN OBJECT IS GREEN"<<endl;
        cout<<greenCount<<endl;
        hconcat(frame, frameHSV, frameHSV_green);
        //imshow("Green Output", frameHSV_green);
    }
    /*else if ((blackCount>redCount)&&(blackCount>blueCount)&&(blackCount>orangeCount)&&(blackCount>yellowCount)&&(blackCount>greenCount)&&(blackCount>whiteCount)&&(blackCount>purpleCount)){

        cout<<"MAIN OBJECT IS BLACK"<<endl;
        cout<<blackCount<<endl;
        hconcat(frame, frameHSV, frameHSV_black);
        //imshow("Black Output", frameHSV_black);
    }*/
    else if ((whiteCount>redCount)&&(whiteCount>blueCount)&&(whiteCount>orangeCount)&&(whiteCount>yellowCount)&&(whiteCount>blackCount)&&(whiteCount>greenCount)&&(whiteCount>purpleCount)){

        cout<<"MAIN OBJECT IS WHITE"<<endl;
        cout<<whiteCount<<endl;
        hconcat(frame, frameHSV, frameHSV_white);
        //imshow("White Output", frameHSV_white);
    }
    else if(((purpleCount>redCount)&&(purpleCount>blueCount)&&(purpleCount>orangeCount)&&(purpleCount>yellowCount)&&(purpleCount>blackCount)&&(purpleCount>greenCount)&&(purpleCount>whiteCount))||(purpleCount>0)){
        cout<<"MAIN OBJECT IS PURPLE"<<endl;
        cout<<purpleCount<<endl;
        hconcat(frame, frameHSV, frameHSV_purple);
        //imshow("Purple Count", frameHSV_purple);
    }

        // Process the image for findContours
        //cvtColor(frame, frame, COLOR_HSV2BGR);
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        adaptiveThreshold(frame, frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 21, 11);
        Mat subFrame = frame(Range(0, 200), Range(0,320));

        int centrePixel = subFrame.cols/2;
        //cout<<centrePixel<<endl;

        //Find the contours
        vector<vector<Point> > contours_blue;
        vector<Vec4i> hierachy_blue;
        Mat contourOutput_blue = subFrame.clone();
        findContours(contourOutput_blue, contours_blue, RETR_LIST, CHAIN_APPROX_SIMPLE);
       // destroyAllWindows();

        //Draw the contours
        Mat contourFrame_blue(subFrame.size(), CV_8UC3, Scalar(0,0,0));
        Scalar Blue[3];
        Blue[0] = Scalar(255, 255,255);
        Blue[1] = Scalar(255,255,255);
        Blue[2] = Scalar(255,255,255);

        for (size_t i = 0; i < contours_blue.size(); i++){
        drawContours(contourFrame_blue, contours_blue, i, Blue[i % 3]);
        //Mat spotFilter = getStructuringElement(MORPH_ELLIPSE, Size(5,5));
        //erode(subFrame, subFrame, spotFilter)        ;
        /*for (vector<Point> > contours_blue){
            for(Point k:p){*/
                //circle(contourFrame_blue, contours_blue, 10, Scalar(255,255,255), FILLED);
            //}}
        float area = contourArea(contours_blue[i]); // Calculatethe area of a contour
        //cout<<area<<endl;

        Point regionCentre = findContourCentre(contours_blue[i]);
        circle(subFrame, regionCentre, 5, Scalar(0,255,0), -1, 8, 0);

        /*cout<<"Region Centre = "<<regionCentre<<endl;
        cout<<"Centre Pixel = " << centrePixel <<endl;*/
        //printf("Contour centre: x = %dpx, y = %dpx\n", regionCentre.x, regionCentre.y);
        }

        vector<vector<Point> > approxedcontours(contours_blue.size()); // Array fr new countours

        for (int i = 0; i < contours_blue.size(); i++){
        approxPolyDP(contours_blue[i],approxedcontours[i], 10, true); // Approximate the contour

        }

       // if(contours_blue.size()>0){
            //Rect bound = boundingRect(contours_blue);
          //  line(contourFrame_blue, Point(bound.x + (bound.width/2),200), Point(bound.x + (bound.width/2),250), Scalar(255,0,0),3);
        //}

        imshow("Contours", contourOutput_blue);
        Mat Transformedframe = contourFrame_blue;
        Rect vertices = boundingRect(contours_blue);
        transformPerspective(contours_blue, Transformedframe, 320, 240);

        Mat Comparison;
        matchTemplate(Transformedframe, Circle, Comparison, TM_CCORR_NORMED, CV_8UC3); // Do the matching and normalise
        normalize(Comparison, Comparison, 0, 1, NORM_MINMAX, -1, CV_8UC3);

        imshow("Transformed Contours", Transformedframe);

        //moveWindow("Contours", 400, 100);

        /*for (int x = max_sensorColumn; x>= 0; x--){
        int pixelValue = frame.at<char>(x,0);
        int pixelWeighting = pixelValue * (x-160);
        }

        weightedAverage = centrePixel; // weighted average sum
        Error= regionCentre.x - weightedAverage;
        P =Error;
        I = I + Error

        if (currentError != previousError){
        u=Kp*currentError+Ki*totalError+Kd*(currentError-previousError); //PID calculations
        totalError+=currentError;
        currentError=previousError;}

        else{
        leftMotor_speed=0; //fail safe mode
        rightMotor_speed=0;
        servoAngle=centreAngle;}

        float k=0.5;
        servoAngle=centreAngle+u; // value updates
        leftMotor_speed=leftSpeed_setting+k*u;
        rightMotor_speed=rightSpeed_setting-k*u;*/


        float error = 0;
        float previousError = 0;


        if(regionCentre.x != 0){
            error = regionCentre.x - centrePixel;}

        float errorCount = errorCount + error;
        float u = (Kp*error)+(Ki*errorCount)+(Kd*(error-previousError));
        previousError = error;
        error = 0;
        //cout<<"U = " << u << endl;
        /*cout<<"Error = " << error << endl;
        cout<<"Error count = " << errorCount << endl;
        cout<<"Previous Error = " << previousError << endl;*/

        leftMotor_speed = leftSpeed_setting+(k*u);
        rightMotor_speed = rightMotor_speed-(k*u);

        if(u > 45){
        u = 45;}
        else if(u < -45){
        u = -45;}
        servoAngle = centreAngle + u;

        char Packet[6];

        Packet[0] = leftMotor_speed >> 8;
        Packet[1] = leftMotor_speed;
        Packet[2] = rightMotor_speed>>8;
        Packet[3] = rightMotor_speed;
        Packet[4] = servoAngle>>8;
        Packet[5] = servoAngle;
        car.i2cWrite(Packet, 6);

        int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)

        key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
        if (key == 27)
            break;
	}

	closeCV();  // Disable the camera and close any windows

	return 0;
}



