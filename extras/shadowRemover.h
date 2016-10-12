#include "laneDetector_utils.h"

Mat removeShadow(Mat img)
{
	Mat edges, img_ycrcb, shadow_rem, shadow, ycrcb_channels[3];
	Mat canny_old, canny_new;

	cvtColor(img, img_ycrcb, CV_BGR2YCrCb);
	split(img,ycrcb_channels);
	shadow=ycrcb_channels[0];
	imshow("shadows", shadow);
	ycrcb_channels[0]=Mat::zeros(img.rows, img.cols, CV_8UC1);
	merge(ycrcb_channels, 3, shadow_rem);

	//canny_old=find_edges(img);
	//canny_new=find_edges(shadow_rem);

	imshow("old", canny_old);
	imshow("now", canny_new);


	imshow("shadow removed", shadow_rem);
	imshow("original", img);
	waitKey(0);

	return shadow_rem;
}