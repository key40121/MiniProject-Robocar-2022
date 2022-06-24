#include <cv.h>
#include <highgui.h>
#include <ctype.h>
#include <iostream>
#include <opencv2/opencv.hpp>

int main ()
{
	// this is a test.

	std::cout << "This is a test." << std::endl;

	cv::Mat img;
	img = cv::imread("images.png");
	
	cv::imshow("test", img);
	cv::waitKey(0);
	
	return 0;
}

