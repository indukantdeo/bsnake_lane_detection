#include"opencv/cv.h"
#include<opencv2/highgui/highgui.hpp>
#include<bits/stdc++.h>
#include "lane_detector.h"

using namespace cv;
using namespace std;

int lowThreshold=70;
int highThreshold=150;

void extract_segments(Mat img_segments[], Mat img,int segments[], int n_segments);

Mat find_edges(Mat img);

int main()
{
	//initialize important variables here
	int n_segments=5;
	int segments[5]={300, 230, 195, 120, 55};
	Mat img_segments[5];

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
		edges=find_edges(img);
		imshow("edges", edges);

		char c=(char)waitKey(10);
		if(c=='q') 
			break;
	}

	extract_segments(img_segments, edges, segments, n_segments);

	vector<Vec4i> lines[n_segments];
	for(i=0; i<n_segments ;i++)
		HoughLinesP(lines[i], lines[i], 1, CV_PI/180, 50, 50, 10 );

  

	imshow("edges", img);
	waitKey(0);

	return 0;
}