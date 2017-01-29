#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

extern int lowThreshold;
extern int highThreshold;

int findIntersection(Vec4i l1, Vec4i l2)
{
	double m1, m2, c1, c2;

	m1=((double)l1[3]-l1[1])/((double)l1[2]-l1[0]);
	c1=(double)l1[3]-m1*l1[2];

	m2=((double)l2[3]-l2[1])/((double)l2[2]-l2[0]);
	c2=(double)l2[3]-m2*l2[2];

	double yi, xi;

	xi=(c1-c2)/(m2-m1);
	yi=m2*xi+c2;

	//if lines are parallel and on road surface but orientation not correct
	if(int(xi)<-200 || int(xi)>1200)
		return 10000;

	return (int)yi;
}

void extractSegments(Mat img_segments[], Mat img,int segments[], int n_segments)
{
	int i, cum_length=0;

	img_segments[0]=img(cv::Rect(0, 0, 1000, segments[0])).clone();
	cum_length+=segments[0];

	for(i=1;i<=n_segments-2;i++)
	{
		img_segments[i]=img(cv::Rect(0, cum_length, 1000, segments[i])).clone();
		cum_length+=segments[i];
	}

	img_segments[n_segments-1]=img(cv::Rect(0, cum_length, 1000, 1000-cum_length)).clone();
	return;
}

void mergeSegments(Mat img_segments[],Mat img, int segments[], int n_segments)
{
	int i, cum_length=0;
	img_segments[0].copyTo(img(cv::Rect(0, 0, 1000, segments[0])));
	cum_length+=segments[0];

	for(i=1;i<=n_segments-2;i++)
	{
		img_segments[i].copyTo(img(cv::Rect(0, cum_length, 1000, segments[i])));
		cum_length+=segments[i];
	}

	img_segments[n_segments-1].copyTo(img(cv::Rect(0, cum_length, 1000, 1000-cum_length)));
	return;
}

Mat findEdges(Mat img)
{
	//directly in single channel
	Mat edges, img_ycrcb, shadow_rem, shadow, ycrcb_channels[3];

	/*cvtColor(img, img_ycrcb, CV_BGR2YCrCb);
	split(img,ycrcb_channels);
	shadow=ycrcb_channels[0];
	imshow("shadows", shadow);
	ycrcb_channels[0]=Mat::zeros(img.rows, img.cols, CV_8UC1);
	merge(ycrcb_channels, 3, shadow_rem);
	imshow("shadow removed", shadow_rem);
	imshow("original", img);*/


	cvtColor(img, img, CV_BGR2GRAY);
	imshow("bw", img);
	medianBlur(img, img, 9);
	//blur(img,img,Size(3,3));
	Canny( img, edges, lowThreshold, highThreshold, 3 );
	imshow("edges", edges);

	//waitKey(0);

	return edges;
}

void getCenterLanes(int segment_h, int segment_len, Point &control_point, Vector<Vec4i> lane_lines, Vector<Vec4i> center_lane_lines,int &lane_center)
{
	cout<<"segment dimension: "<<segment_h<<"x"<<segment_len<<"  lane center:"<<lane_center<<endl;
	cout<<"lane_lines size: "<<lane_lines.size()<<endl;
	int i;
	Vector<int> right_lane_centers, left_lane_centers;
	for(i=0;i<lane_lines.size();i++)
	{
		if( lane_lines[i][0]>lane_center && lane_lines[i][2]>lane_center)
		{
			right_lane_centers.push_back( (lane_lines[i][0]+lane_lines[i][2])/2  );
		}
		else if( lane_lines[i][0]<lane_center && lane_lines[i][2]<lane_center )
		{
			left_lane_centers.push_back( (lane_lines[i][0]+lane_lines[i][2])/2 );
		}			
	}
	sort(right_lane_centers.begin(), right_lane_centers.end());
	sort(left_lane_centers.begin(), left_lane_centers.end());
	reverse(left_lane_centers.begin(), left_lane_centers.end());

	int right_lane_center=right_lane_centers[0], left_lane_center=left_lane_centers[0];
	cout<<"right_lane_center "<<right_lane_centers[0]<<" left_lane_center "<<left_lane_centers[0]<<endl;
	double top_right=0, top_left=0;
	int right_lines=0, left_lines=0;

	int curve_threshold=50;
	for(i=0;i<lane_lines.size();i++)
	{
		if( lane_lines[i][0]>lane_center && lane_lines[i][2]>lane_center)
		{
			if( (lane_lines[i][0]+lane_lines[i][2])/2 < right_lane_center + curve_threshold )
			{
				center_lane_lines.push_back(lane_lines[i]);
				right_lines++;
				top_right+=(lane_lines[i][2]-lane_lines[i][0])*(-lane_lines[i][1])/(lane_lines[i][3]-lane_lines[i][1])+lane_lines[i][0];
			}
		}
		else if( lane_lines[i][0]<lane_center && lane_lines[i][2]<lane_center )
		{
			if( (lane_lines[i][0]+lane_lines[i][2])/2 > left_lane_center - curve_threshold )
			{
				center_lane_lines.push_back(lane_lines[i]);
				left_lines++;
				top_left+=(lane_lines[i][2]-lane_lines[i][0])*(-lane_lines[i][1])/(lane_lines[i][3]-lane_lines[i][1])+lane_lines[i][0];
			}
		}			
	}

	top_right=(int)top_right/right_lines;
	top_left=(int)top_left/left_lines;

	lane_center=(top_left+top_right)/2;
	control_point.x=lane_center;

	cout<<"lane center: "<<lane_center<<endl;

	return;
		
}