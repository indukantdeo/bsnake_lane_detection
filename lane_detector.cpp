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
	int segments[5]={75, 140, 215, 250, 320};
	Mat img_segments[5];
	std::stringstream window_name;

	cv::Mat img=cv::imread("images/l3.jpg", CV_LOAD_IMAGE_COLOR);
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

	int i, j, k;

	/*display edge segments
	for(i=0;i<n_segments;i++)
		cout<<img_segments[i].rows<<" "<<img_segments[i].cols<<endl;
	
	for(i=0;i<n_segments;i++)
  	{
  		window_name<<"edges"<<i<<"";
  		imshow(window_name.str(), img_segments[i]);
  		window_name.str("");
  	}*/

	vector<Vec4i> lines[n_segments];
	int hough_threshold[5]={30, 30, 30, 50, 100};
	int hough_minLineLength[5]={20, 25, 25, 30, 80};
	for(i=0; i<n_segments ;i++)
		HoughLinesP(img_segments[i], lines[i], 1, CV_PI/180, hough_threshold[i], hough_minLineLength[i], 50 );

  	Mat line_segments[n_segments];
  	Mat empty=img-img;
  	extract_segments(line_segments, empty, segments, n_segments);


  	for(i=0;i<n_segments;i++)
  		for(j=0;j<lines[i].size();j++)
  		{
  			Vec4i l = lines[i][j];
  			line( line_segments[i], Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  		}


  	/* display line segments
  	for(i=0;i<n_segments;i++)
  	{
  		window_name<<"window"<<i<<"";
  		imshow(window_name.str(), line_segments[i]);
  		window_name.str("");
  	}*/


  	Mat line=img-img;
  	merge_segments(line_segments, line, segments, n_segments);
  	imshow("detected lines",line);



	imshow("edges", edges);
	waitKey(0);

	return 0;
}