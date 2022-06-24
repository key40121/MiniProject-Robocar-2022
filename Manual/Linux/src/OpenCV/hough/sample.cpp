#include <unistd.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
//#include <cv.h>
//#include <highgui.h>
#include <math.h>

int
main (int argc, char **argv)
{
	int i;
	float *line, rho, theta;
	double a, b, x0, y0;
	CvMemStorage *storage;
	CvSeq *lines = 0;
	CvPoint *point, pt1, pt2;

	CvCapture *videoCapture = cvCreateCameraCapture(0);

	IplImage *image;
	IplImage *gray;
	IplImage *prob;
	while(1){
		image = cvQueryFrame(videoCapture);
//		cvNamedWindow ("Hough_line_standard", CV_WINDOW_AUTOSIZE);
//		cvShowImage ("Hough_line_standard", image);
		gray = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
		cvCvtColor(image, gray, CV_BGR2GRAY);
		prob = cvCreateImage(cvGetSize(gray), IPL_DEPTH_8U, 1);
		prob = cvCloneImage(gray);

		// (2)ハフ変換のための前処理
		cvCanny (gray, gray, 50, 200, 3);
		storage = cvCreateMemStorage (0);

		// (3)標準的ハフ変換による線の検出と検出した線の描画
// 		lines = cvHoughLines2 (gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 30, 5);
 		lines = cvHoughLines2(gray,storage,CV_HOUGH_STANDARD,1,CV_PI/180,50,0,0);
/*		for (i = 0; i < MIN (lines->total, 10); i++) {
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
			cvLine (image, pt1, pt2, CV_RGB (255, 0, 0), 3, 8, 0);
		}
*/
		// (4)確率的ハフ変換による線分の検出と検出した線分の描画
		lines = 0;
		lines = cvHoughLines2 (gray, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, 50, 50, 10);

		for (i = 0; i < MIN (lines->total, 10); i++) {
//		for (i = 0; i < lines->total; i++) {
			point = (CvPoint *) cvGetSeqElem (lines, i);
			cvLine (prob, point[0], point[1], CV_RGB (255, 0, 0), 3, 8, 0);
		}

		// (5)検出結果表示用のウィンドウを確保し表示する
		cvNamedWindow ("Hough_line_probalistic", CV_WINDOW_AUTOSIZE);
		cvShowImage ("Hough_line_probalistic", prob);
		int ret = cvWaitKey (1);
		if(ret > 0)
			break;
	}
	cvDestroyWindow ("Hough_line_standard");
	cvDestroyWindow ("Hough_line_standard");
	cvReleaseImage (&image);
	cvReleaseImage (&prob);
	cvReleaseImage (&gray);
	cvReleaseMemStorage (&storage);
	return 0;
}

