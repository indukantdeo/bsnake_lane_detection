#ifndef _LANEDETECTOR_UTILS_HPP_
#define _LANEDETECTOR_UTILS_HPP_

#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

extern int lowThreshold;
extern int highThreshold;

int findIntersection(Vec4i l1, Vec4i l2);
void extractSegments(Mat img_segments[], Mat img,int segments[], int n_segments);
void mergeSegments(Mat img_segments[],Mat img, int segments[], int n_segments);
Mat findEdges(Mat img);
void getCenterLanes(int segment_h, int segment_len, Point &control_point, Vector<Vec4i> lane_lines, Vector<Vec4i> center_lane_lines,int &lane_center);
void drawBezierSpline(Mat &img, Point control_points[], int num_control_points, int n_segments);

#endif


/*
Sign Conventions:

findIntersections: gives the distance of the intersection of the two lines from the top of the segment

cv::Line takes points as
--------->	first co-ordinate
|
|
|
|
|
second co-ordinate

*/