#include <unistd.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include <math.h>

void hough_standard();
void hough_probalistic();

CvMemStorage *storage;
CvSeq *lines = 0;
CvCapture *videoCapture = cvCreateCameraCapture(0);
IplImage *image;
IplImage *gray;
IplImage *res;

int main (int argc, char **argv)
{
	while(1){
		storage = cvCreateMemStorage (0);
		image = cvQueryFrame(videoCapture);
		gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

		//標準的Hough変換
//		hough_standard();
		//確率的Hough変換
		hough_probalistic();
	
		int ret = cvWaitKey (1);
		if(ret > 0)
			break;
	}

	cvDestroyWindow ("Hough_line_result");
	cvDestroyWindow ("Hough_line_result");
	cvReleaseImage (&image);
	cvReleaseImage (&res);
	cvReleaseImage (&gray);
	cvReleaseMemStorage (&storage);
	return 0;
}

void hough_standard(){

	int i;
	float *line, rho, theta;
	double a, b, x0, y0;
	CvPoint pt1, pt2;
	res = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);

	// Canny変換
	cvCanny (image, gray, 50, 200, 3);
	cvCvtColor(gray, res, CV_GRAY2BGR);
	storage = cvCreateMemStorage(0);

	lines = cvHoughLines2(gray,storage,CV_HOUGH_STANDARD, 1, CV_PI/180, 50, 0, 0);
	for (i = 0; i < MIN(lines->total, 10); i++) {
		line = (float *) cvGetSeqElem (lines, i);
		rho = line[0];
		theta = line[1];
		a = cos (theta);
		b = sin (theta);
		x0 = a * rho;
		y0 = b * rho;
		pt1.x = cvRound (x0 + 1000 * (-b));
		pt1.y = cvRound (y0 + 1000 * (a));
		pt2.x = cvRound (x0 - 1000 * (-b));
		pt2.y = cvRound (y0 - 1000 * (a));
		cvLine(res, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
	}

	// 検出結果表示用のウィンドウを確保し表示する
	cvNamedWindow("Hough_line_result", CV_WINDOW_AUTOSIZE);
	cvShowImage("Hough_line_result", res);

}

void hough_probalistic(){

	int i;
	CvPoint *point;
	res = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);

	// Canny変換
	cvCanny (image, gray, 50, 200, 3);
	cvCvtColor(gray, res, CV_GRAY2BGR);
	storage = cvCreateMemStorage(0);

	lines = 0;
	lines = cvHoughLines2(gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, 50, 10);

	for (i = 0; i < MIN(lines->total, 10); i++) {
		point = (CvPoint *) cvGetSeqElem (lines, i);
		cvLine(res, point[0], point[1], CV_RGB(255, 0, 0), 3, 8, 0);
	}

	// 検出結果表示用のウィンドウを確保し表示する
	cvNamedWindow("Hough_line_result", CV_WINDOW_AUTOSIZE);
	cvShowImage("Hough_line_result", res);
}
