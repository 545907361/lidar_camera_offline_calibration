/*
This is a demo application to showcase the trackbar component.
Authors: Pascal Thomet, Fernando Bevilacqua
*/

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define WINDOW_NAME	"Trackbar"

int main(int argc, const char *argv[])
{
			int hl = 60;
    int sl = 25;
    int vl = 25;
    int hh = 130;
    int sh = 255;
    int vh = 255;
    int contours_num = 200;

    float zl = -1.0;
    float zh =  1.0;
    float xl = -2.0;
    float xh =  1.5;
    float yl =  2.0;
    float yh =  5.0;
    float ball_r = 30.0;
cvui::init("panel");
	cv::Mat frame = cv::Mat(690, 710, CV_8UC3);

	// The width of all trackbars used in this example.
	int width = 300;

	// The x position of all trackbars used in this example
	int x = 10;

	// Init cvui and tell it to create a OpenCV window, i.e. cv::namedWindow(WINDOW_NAME).

	while (true) {
		// Fill the frame with a nice color
		frame = cv::Scalar(49, 52, 49);

		// The trackbar component uses templates to guess the type of its arguments.
		// You have to be very explicit about the type of the value, the min and
		// the max params. For instance, if they are double, use 100.0 instead of 100.

    frame = cv::Scalar(49, 52, 49);
    
    cvui::text(frame, 10, 20, "image config");
    cvui::text(frame, 10, 50, "hl");
    cvui::trackbar(frame, 10, 80, 300, &hl, 0, 180, 3);
    cvui::text(frame, 10, 140, "hh");
    cvui::trackbar(frame, 10, 170, 300, &hh, 0, 180, 3);
    cvui::text(frame, 10, 230, "sl");
    cvui::trackbar(frame, 10, 260, 300, &sl, 0, 255);
    cvui::text(frame, 10, 320, "sh");
    cvui::trackbar(frame, 10, 350, 300, &sh, 0, 255);
    cvui::text(frame, 10, 410, "vl");
    cvui::trackbar(frame, 10, 440, 300, &vl, 0, 255);
    cvui::text(frame, 10, 500, "vh");
    cvui::trackbar(frame, 10, 530, 300, &vh, 0, 255);
    cvui::text(frame, 10, 590, "contours_num");
    cvui::trackbar(frame, 10, 620, 300, &contours_num, 0, 1000, 5);

    cvui::text(frame, 400, 20, "pointcloud config");
    cvui::text(frame, 400, 50, "xl");
    cvui::trackbar(frame, 400, 80, 300, &xl, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 140, "xh");
    cvui::trackbar(frame, 400, 170, 300, &xh, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 230, "yl");
    cvui::trackbar(frame, 400, 260, 300, &yl, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 320, "yh");
    cvui::trackbar(frame, 400, 350, 300, &yh, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 410, "zl");
    cvui::trackbar(frame, 400, 440, 300, &zl, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 500, "zh");
    cvui::trackbar(frame, 400, 530, 300, &zh, (float)-15.0, (float)15.0, 6);
    cvui::text(frame, 400, 590, "ball_r");
    cvui::trackbar(frame, 400, 620, 300, &ball_r, (float)0.0, (float)50.0, 5);


		// This function must be called *AFTER* all UI components. It does
		// all the behind the scenes magic to handle mouse clicks, etc.
		cvui::update();

		// Show everything on the screen
		cv::imshow("panel", frame);

		// Check if ESC key was pressed
		if (cv::waitKey(1000) == 27) {
			break;
		}
	}

	return 0;
}
