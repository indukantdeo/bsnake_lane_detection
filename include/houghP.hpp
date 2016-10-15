#ifndef _HOUGHP_HPP_
#define _HOUGHP_HPP_

#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include <bits/stdc++.h>

using namespace std;
using namespace cv;

void HoughLinesP2( Mat& image,
                         float rho, float theta, int threshold,
                         int lineLength, int lineGap,
                         std::vector<Vec4i>& lines, std::vector<int>& len, int linesMax );

#endif