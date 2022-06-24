//#include <cv.h>
//#include <highgui.h>
#include "opencv2/opencv.hpp" 
#include <stdio.h>
#include <math.h>

int main(int argc, char** argv)
{
	CvCapture *videoCapture = cvCreateCameraCapture(0);
    IplImage *src;
	IplImage *dst;
	IplImage *color_dst;
	CvMemStorage *storage;

	while(1)
    {
		src = cvQueryFrame(videoCapture);
#if 0
        cvNamedWindow( "Source", 1 );
        cvShowImage( "Source", src );
#endif
        dst = cvCreateImage( cvGetSize(src), 8, 1 );
        color_dst = cvCreateImage( cvGetSize(src), 8, 3 );
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* lines = 0;
        int i;
        cvCanny( src, dst, 50, 200, 3 );
        cvCvtColor( dst, color_dst, CV_GRAY2BGR );
#if 0 
        lines = cvHoughLines2( dst, storage, CV_HOUGH_STANDARD, 1, CV_PI/180, 100, 0, 0 );

        for( i = 0; i < MIN(lines->total,30); i++ )
        {
            float* line = (float*)cvGetSeqElem(lines,i);
            float rho = line[0];
            float theta = line[1];
            CvPoint pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            cvLine( color_dst, pt1, pt2, CV_RGB(255,0,0), 3, 4 );
        }
#endif

#if 1 
        lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 70, 70, 10 );
        for( i = 0; i < lines->total; i++ )
//        for( i = 0; i < 2; i++ )
        {
            CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
#if 0
			printf("i=%d x0=%d y0=%d\n", i, line[0].x, line[0].y);
			printf("i=%d x1=%d y1=%d\n", i, line[1].x, line[1].y);
#endif
			if(line[0].y > 240) {
				if(line[0].x < 320){
					if(line[0].y > line[1].y){ 
	            		cvLine( color_dst, line[0], line[1], CV_RGB(255,0,0), 3, 8 );
					}
				} else {
					if(line[0].y < line[1].y){ 
	            		cvLine( color_dst, line[0], line[1], CV_RGB(255,0,0), 3, 8 );
					}
				}
			}
        }
        cvNamedWindow( "Hough", 1 );
        cvShowImage( "Hough", color_dst );
#endif


//        cvWaitKey(0);
        if(cvWaitKey(1) > 0)
			break;
    }
	cvReleaseImage(&src);
	cvReleaseImage(&dst);
	cvReleaseImage(&color_dst);
	cvReleaseMemStorage(&storage);
}
