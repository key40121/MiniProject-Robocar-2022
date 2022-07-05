#include <cv.h>
#include <highgui.h>
#include <ctype.h>

// for line detecton and image processing
#include <iostream>
#include <tuple>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// robocar dynamics
#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "RcControl.h"

using namespace zmp::zrc;


// ver5
double ymm_per_pix = 310 / 240;
double xmm_per_pix = 580 / 160;

// functions for calculation
std::vector<cv::Point2f> SlidingWindow(cv::Mat, cv::Rect);
std::tuple<double, double> Polynomial(std::vector<cv::Point2f>);
double SteerAngle(double);
bool WhiteLaneDetection(cv::Mat);
int white_lane_detection_pix(cv::Mat);

// functions for image processing
cv::Mat ImageCalibration(cv::Mat);
cv::Mat ImageBirdsEyeProcess(cv::Mat);
cv::Mat ImageProcessing(cv::Mat);
std::tuple<cv::Mat, cv::Mat> CalibMatrix();
std::vector<cv::Point2f> FindNonZero(cv::Mat);


int main() {
	// Robocar dynamics
    char buf[20];
    float angle = 0;
    int speed = 0;
    int set_v = 0;
    int set_t = 0;
    bool sign = false;
    float set_a = 0;
    bool period = false;
    bool loop = true;
    RcControl  _RcControl;
    _RcControl.init();
    _RcControl.Start();

    _RcControl.SetReportFlagReq(0x0f);
    _RcControl.SetServoEnable(1);
    _RcControl.SetMotorEnableReq(1);
    _RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);
	
    _RcControl.SetDriveSpeed(300);


	// Image processing and line detection
	cv::VideoCapture cap(0);
	
	const int HEIGHT = 240;
	const int WIDTH = 320;
	const int FPS = 30;
	
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	cap.set(CV_CAP_PROP_FPS, FPS);

	// if radius of curvature is less than the height of robocar, it would be nan.
	int steering_angle_restore;
	std::vector<int> steering_angle_list;

	// make sure camera is loaded
	if(!cap.isOpened())
	{
		return -1;
	}
	
	cv::Mat frame; 
	while(cap.read(frame))
	{
	    //
	    //Processing.
	    //
		cv::Mat img;
		cv::Mat processed;

		img = frame;

		// calibration
		processed = ImageCalibration(img);

		// Perspective
		processed = ImageBirdsEyeProcess(processed);

		// Image processing
		processed = ImageProcessing(processed);

		// sliding window algorithm
		// ver5 the center is 106
		std::vector<cv::Point2f> pts = SlidingWindow(processed, cv::Rect(50, 210, 100, 30));


		// Polynomial fitting
		double poly_co, lin_co;
		std::tie(poly_co, lin_co) = Polynomial(pts);

		// Radius of curvature
		double radius_of_curvature;
		radius_of_curvature = std::pow((1 + std::pow(2 * poly_co * pts[1].x * xmm_per_pix + lin_co, 2)), 1.5) / std::abs(2 * poly_co);

		std::cout << "--------------------------------" << std::endl;
		std::cout << "Radius of curvature is " << radius_of_curvature << std::endl;

		// have to think about the width of the car and the lane.
		// radius_of_curvature = radius_of_curvature - 182.5;
		radius_of_curvature = radius_of_curvature - 277.5;
	
		// steering angle
		double steering_angle;

		if (radius_of_curvature <= 429 && radius_of_curvature >= -429)
		{
			steering_angle = steering_angle_restore;
		}
		else
		{
			steering_angle = std::round(SteerAngle(radius_of_curvature));
			steering_angle_restore = steering_angle;
		}

		// lane centering (needs to do test to set proper amount of the number)
		// the center would be 53?
		
		if (pts[0].x >= 116)
		{
			steering_angle = steering_angle + 2;
		}
		else if (pts[0].x <= 96)
		{
			steering_angle = steering_angle - 2;
		}
		else if (pts[0].x <= 60)
		{
			steering_angle = steering_angle + 5;
		}
		

		if (steering_angle >= 30)
		{
			steering_angle = 30;
		} 
		else if (steering_angle <= -30)
		{
			steering_angle = -30;
		}

		// lane centering for straight line.
		bool white_existance = WhiteLaneDetection(processed);
		if (white_existance == false)
		{
			steering_angle = steering_angle + 3;
		}

		steering_angle = steering_angle;
	
		std::cout << "Steering angle is " << steering_angle << std::endl;
		std::cout << "-------------------------------" << std::endl;
	
		/*--------------------------------------------------------------------*/	
		// vechile dynamics
		_RcControl.SetSteerAngle(steering_angle);
		/*--------------------------------------------------------------------*/
	
		//cv::imshow("win", frame);
		const int key = cv::waitKey(1);
		if(key == 'q'/*113*/)
		{
			break;
		}
		else if(key == 's'/*115*/)
		{
			cv::imwrite("img.png", frame);
		}
	}


	cv::destroyAllWindows();
	return 0;
}

std::vector<cv::Point2f> FindNonZero(cv::Mat img)
{
	std::vector<cv::Point2f> locations;
	int width = img.cols;
	int height = img.rows;

	for (int i = 0; i < img.cols; i++)
	{
		for (int j = 0; j < img.rows; j++)
		{
			int intensity = img.at<unsigned char>(j, i);
			if (intensity == 255)
			{
				locations.push_back(cv::Point2f(i, j));
			}
		}
	}

	return locations;
}

std::vector<cv::Point2f> SlidingWindow(cv::Mat image, cv::Rect window) {
	std::vector<cv::Point2f> points;
	const cv::Size imgSize = image.size();
	bool shouldBreak = false;

	//test 
	//cv::Point2f start = cv::Point2f(77, 240);
	//points.push_back(start);

	while (true)
	{
		float currentX = window.x + window.width * 0.5f;

		// Extract region of interest.
		cv::Mat roi;
		roi = cv::Mat(image, window);

		std::vector<cv::Point2f> locations;

		//--------------------------------------------------------------------//
		// Get all non-black pixels. All pixels are white in our case.
		// cannot use cv::findNonzero on robocar so I made FindNonZero by myself.
		// cv::findNonZero(roi, locations);
		locations = FindNonZero(roi);
		// ------------------------------------------------//

		float avgX = 0.0f;

		// Calculate average X position
		for (int i = 0; i < locations.size(); ++i)
		{
			float x = locations[i].x;
			avgX += window.x + x;
		}

		// sankou enzanshi
		avgX = locations.empty() ? currentX : avgX / locations.size();

		cv::Point point(avgX, window.y + window.height * 0.5f);
		points.push_back(point);

		// Move the window up
		window.y -= window.height;

		// For the uppermost position
		if (window.y < 0)
		{
			window.y = 0;
			shouldBreak = true;
		}

		// Move the x position
		window.x += (point.x - currentX);

		// Make sure the window dosent overflow, we get an error if we try to fet data outside the mattrix
		if (window.x < 0)
		{
			window.x = 0;
		}

		if (window.x + window.width >= imgSize.width)
		{
			window.x = imgSize.width - window.width - 1;
		}

		if (shouldBreak)
		{
			break;
		}
	}
	return points;
}

std::tuple<double, double> Polynomial(std::vector<cv::Point2f> pts)
{
	int i, j, k, n, N;


	// The number of the point

	N = 8;
	double x[8];
	double y[8];

	// N = 8;
	// double x[8];
	// double y[8];

	for (int i = 0; i < pts.size(); i++)
	{
		x[i] = pts[i].x * xmm_per_pix;
		y[i] = pts[i].y * ymm_per_pix;
	}


	//std::cout << "\nEnter the x-axis values:\n";                //Input x-values
	//for (i = 0; i < N; i++)
	//	std::cin >> x[i];
	//std::cout << "\nEnter the y-axis values:\n";                //Input y-values
	//for (i = 0; i < N; i++)
	//	std::cin >> y[i];

	// n is the degree of Polynomial 
	n = 2;

	// 

	double X[5];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	for (i = 0; i < 2 * n + 1; i++)
	{
		X[i] = 0;
		for (j = 0; j < N; j++)
			X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
	}
	double B[3][4], a[3];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
	for (i = 0; i <= n; i++)
		for (j = 0; j <= n; j++)
			B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
	double Y[3];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	for (i = 0; i < n + 1; i++)
	{
		Y[i] = 0;
		for (j = 0; j < N; j++)
			Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
	}
	for (i = 0; i <= n; i++)
		B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
	n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
	//std::cout << "\nThe Normal(Augmented Matrix) is as follows:\n";
	//for (i = 0; i < n; i++)            //print the Normal-augmented matrix
	//{
	//	for (j = 0; j <= n; j++)
	//		std::cout << B[i][j] << std::setw(16);
	//	std::cout << "\n";
	//}
	for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
		for (k = i + 1; k < n; k++)
			if (B[i][i] < B[k][i])
				for (j = 0; j <= n; j++)
				{
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}

	for (i = 0; i < n - 1; i++)            //loop to perform the gauss elimination
		for (k = i + 1; k < n; k++)
		{
			double t = B[k][i] / B[i][i];
			for (j = 0; j <= n; j++)
				B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
		}
	for (i = n - 1; i >= 0; i--)                //back-substitution
	{                        //x is an array whose values correspond to the values of x,y,z..
		a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
		for (j = 0; j < n; j++)
			if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
				a[i] = a[i] - B[i][j] * a[j];
		a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
	}
	std::cout << "\nThe values of the coefficients are as follows:\n";
	for (i = 0; i < n; i++)
		std::cout << "x^" << i << "=" << a[i] << std::endl;            // Print the values of x^0,x^1,x^2,x^3,....    
	std::cout << "\nHence the fitted Polynomial is given by:\ny=";
	for (i = 0; i < n; i++)
		std::cout << " + (" << a[i] << ")" << "x^" << i;
	std::cout << "\n";


	return std::forward_as_tuple(a[2], a[1]);
}

std::tuple<cv::Mat, cv::Mat> CalibMatrix()
{
	// call calibration using distortion matrix
	cv::Mat camera_matrix;
	cv::Mat distortion_coeff;
	
	cv::FileStorage fs("calibration_matrix.xml", cv::FileStorage::READ);
	fs["intrinsic"] >> camera_matrix;
	fs["distortion"] >> distortion_coeff;
	fs.release();

	return std::forward_as_tuple(camera_matrix, distortion_coeff);
}

cv::Mat ImageCalibration(cv::Mat img)
{
	// calibration using distortion matrix
	cv::Mat camera_matrix;
	cv::Mat distortion_coeff;

	cv::Mat undistort_img;

	std::tie(camera_matrix, distortion_coeff) = CalibMatrix();

	cv::undistort(img, undistort_img, camera_matrix, distortion_coeff);

	return undistort_img;
}

cv::Mat ImageBirdsEyeProcess(cv::Mat img)
{
	// rectangles ver5
	cv::Point2f srcVertices[4];
	srcVertices[0] = cv::Point(77, 63);
	srcVertices[1] = cv::Point(236, 63);
	srcVertices[2] = cv::Point(318, 100);
	srcVertices[3] = cv::Point(0, 100);

	cv::Point2f dstVertices[4];
	dstVertices[0] = cv::Point(80, 0);
	dstVertices[1] = cv::Point(240, 0);
	dstVertices[2] = cv::Point(240, 240);
	dstVertices[3] = cv::Point(80, 240);


	// Prepare matrix for transform and get the warped image
	cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcVertices, dstVertices);
	cv::Mat dst(240, 320, CV_8UC3); // Destination for warped image

	// For transforming back into original space
	cv::Mat invertedPerspectiveMatrix;
	cv::invert(perspectiveMatrix, invertedPerspectiveMatrix);

	cv:warpPerspective(img, dst, perspectiveMatrix, dst.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);

	return dst;
}

cv::Mat ImageProcessing(cv::Mat img)
{
	cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
	
	// just thresholding
	cv::Mat processed;
	const int THRESHOLD_VAL = 190;
	cv::threshold(img, processed, THRESHOLD_VAL, 255, cv::THRESH_BINARY);

	return processed;
}

double SteerAngle(double radius_of_curvature)
{
	double steer_angle;
	const double ROBOCAR_WIDTH = 429;
	const double PI = 3.141592;

	// calculate steer angle using radius of curvature and the width of robocar.
	if (radius_of_curvature > 50000)
	{
		steer_angle = 0;
	}
	else if (radius_of_curvature < -50000)
	{
		steer_angle = 0;
	}
	else 
	{
		steer_angle = std::asin(ROBOCAR_WIDTH / radius_of_curvature) * 360 / 2 / PI;
	}

	if (steer_angle >= 30)
	{
		steer_angle = 30;
	}
	else if (steer_angle <= -30)
	{
		steer_angle = -30;
	}
	
	return steer_angle;
}

bool WhiteLaneDetection(cv::Mat img)
{
	// to see if the car is out of the center or not.
	// if the car is out of the center, the camera would not capture the lane at (x, y) = (50, 230) [px].
	bool white_existance = false;

	for (int i = 20; i < 120; i++)
	{
		int intensity = img.at<unsigned char>(235, i);
		if (intensity == 255)
		{
			white_existance = true;
			break;
		}
	}

	return white_existance;
}

