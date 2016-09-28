#include"opencv/cv.h"
#include<opencv2/highgui/highgui.hpp>
#include<bits/stdc++.h>

using namespace cv;
using namespace std;

int lowThreshold=70;
int highThreshold=150;

Mat find_edges(Mat img, int* segments, int n_segments)
{
	//directly in single channel
	Mat edges;
	cvtColor(img, img, CV_BGR2GRAY);
	imshow("bw", img);
	waitKey(1);
	blur(img,img,Size(3,3));
	Canny( img, edges, lowThreshold, highThreshold, 3 );

	return edges;
}

int main()
{
	//initialize important variables here
	int n_segments=5;
	int segments[6]={300, 230, 195, 120, 55};

	cv::Mat img=cv::imread("images/l2.jpg", CV_LOAD_IMAGE_COLOR);
	cv::resize(img, img, cv::Size(1000,1000));
	imshow("lanes", img);

	int max_lowThreshold=500, max_highThreshold=500;
	namedWindow("Edge threshold", CV_WINDOW_AUTOSIZE);
	createTrackbar( "Min Threshold:", "Edge threshold", &lowThreshold, max_lowThreshold);
	createTrackbar( "Max Threshold:", "Edge threshold", &highThreshold, max_highThreshold);

	Mat edges;
	while(true)
	{
		edges=find_edges(img, segments, n_segments);
		imshow("edges", edges);
		char c=waitKey(10);
		if(c=='q')
			break;
	}

	imshow("edges", img);
	waitKey(0);

	return 0;
}