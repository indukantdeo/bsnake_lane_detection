#include"opencv/cv.h"
#include<opencv2/highgui/highgui.hpp>
#include<bits/stdc++.h>
#include "lane_detector.h"

using namespace cv;
using namespace std;

//constant definitions
#define FLOAT_MAT_TYPE CV_32FC1
#define FLOAT_MAT_ELEM_TYPE float

#define INT_MAT_TYPE CV_8UC1
#define INT_MAT_ELEM_TYPE unsigned char

#define FLOAT_IMAGE_TYPE IPL_DEPTH_32F
#define FLOAT_IMAGE_ELEM_TYPE float

#define INT_IMAGE_TYPE IPL_DEPTH_8U
#define INT_IMAGE_ELEM_TYPE unsigned char

#define FLOAT_POINT2D CvPoint2D32f
#define FLOAT_POINT2D_F cvPoint2D632f

#define FLOAT float
#define INT int
#define SHORT_INT unsigned char


int lowThreshold=70;
int highThreshold=150;

cv::Point2f src_vertices[4];
 static bool done = false;
 int no_clicks=0;

typedef enum LineType_ {
  LINE_HORIZONTAL = 0,
  LINE_VERTICAL = 1
} LineType;



void mcvFilterLines(const CvMat *inImage, CvMat *outImage,
                     unsigned char wx, unsigned char wy, FLOAT sigmax,
                     FLOAT sigmay, LineType lineType)
{
    //define the two kernels
    //this is for 7-pixels wide
//     FLOAT_MAT_ELEM_TYPE derivp[] = {-2.328306e-10, -6.984919e-09, -1.008157e-07, -9.313226e-07, -6.178394e-06, -3.129616e-05, -1.255888e-04, -4.085824e-04, -1.092623e-03, -2.416329e-03, -4.408169e-03, -6.530620e-03, -7.510213e-03, -5.777087e-03, -5.777087e-04, 6.932504e-03, 1.372058e-02, 1.646470e-02, 1.372058e-02, 6.932504e-03, -5.777087e-04, -5.777087e-03, -7.510213e-03, -6.530620e-03, -4.408169e-03, -2.416329e-03, -1.092623e-03, -4.085824e-04, -1.255888e-04, -3.129616e-05, -6.178394e-06, -9.313226e-07, -1.008157e-07, -6.984919e-09, -2.328306e-10};
//     int derivLen = 35;
//     FLOAT_MAT_ELEM_TYPE smoothp[] = {2.384186e-07, 5.245209e-06, 5.507469e-05, 3.671646e-04, 1.744032e-03, 6.278515e-03, 1.778913e-02, 4.066086e-02, 7.623911e-02, 1.185942e-01, 1.541724e-01, 1.681881e-01, 1.541724e-01, 1.185942e-01, 7.623911e-02, 4.066086e-02, 1.778913e-02, 6.278515e-03, 1.744032e-03, 3.671646e-04, 5.507469e-05, 5.245209e-06, 2.384186e-07};
//     int smoothLen = 23;
  CvMat fx;
  CvMat fy;
  //create the convoultion kernel
  switch (lineType)
  {
    case LINE_HORIZONTAL:
    {
      //this is for 5-pixels wide
      FLOAT_MAT_ELEM_TYPE derivp[] = {-2.384186e-07, -4.768372e-06, -4.482269e-05, -2.622604e-04, -1.064777e-03, -3.157616e-03, -6.976128e-03, -1.136112e-02, -1.270652e-02, -6.776810e-03, 6.776810e-03, 2.156258e-02, 2.803135e-02, 2.156258e-02, 6.776810e-03, -6.776810e-03, -1.270652e-02, -1.136112e-02, -6.976128e-03, -3.157616e-03, -1.064777e-03, -2.622604e-04, -4.482269e-05, -4.768372e-06, -2.384186e-07};
      int derivLen = 25;
      FLOAT_MAT_ELEM_TYPE smoothp[] = {2.384186e-07, 5.245209e-06, 5.507469e-05, 3.671646e-04, 1.744032e-03, 6.278515e-03, 1.778913e-02, 4.066086e-02, 7.623911e-02, 1.185942e-01, 1.541724e-01, 1.681881e-01, 1.541724e-01, 1.185942e-01, 7.623911e-02, 4.066086e-02, 1.778913e-02, 6.278515e-03, 1.744032e-03, 3.671646e-04, 5.507469e-05, 5.245209e-06, 2.384186e-07};
      int smoothLen = 23;
      //horizontal is smoothing and vertical is derivative
      fx = cvMat(1, smoothLen, FLOAT_MAT_TYPE, smoothp);
      fy = cvMat(derivLen, 1, FLOAT_MAT_TYPE, derivp);
    }
    break;

    case LINE_VERTICAL:
    {
      //this is for 5-pixels wide
      FLOAT_MAT_ELEM_TYPE derivp[] = //{1.000000e-11, 8.800000e-10, 3.531000e-08, 8.536000e-07, 1.383415e-05, 1.581862e-04, 1.306992e-03, 7.852691e-03, 3.402475e-02, 1.038205e-01, 2.137474e-01, 2.781496e-01, 2.137474e-01, 1.038205e-01, 3.402475e-02, 7.852691e-03, 1.306992e-03, 1.581862e-04, 1.383415e-05, 8.536000e-07, 3.531000e-08, 8.800000e-10, 1.000000e-11};
          //{1.000000e-06, 4.800000e-05, 9.660000e-04, 1.048000e-02, 6.529500e-02, 2.278080e-01, 3.908040e-01, 2.278080e-01, 6.529500e-02, 1.048000e-02, 9.660000e-04, 4.800000e-05, 1.000000e-06};
          {1.000000e-16, 1.280000e-14, 7.696000e-13, 2.886400e-11, 7.562360e-10, 1.468714e-08, 2.189405e-07, 2.558828e-06, 2.374101e-05, 1.759328e-04, 1.042202e-03, 4.915650e-03, 1.829620e-02, 5.297748e-02, 1.169560e-01, 1.918578e-01, 2.275044e-01, 1.918578e-01, 1.169560e-01, 5.297748e-02, 1.829620e-02, 4.915650e-03, 1.042202e-03, 1.759328e-04, 2.374101e-05, 2.558828e-06, 2.189405e-07, 1.468714e-08, 7.562360e-10, 2.886400e-11, 7.696000e-13, 1.280000e-14, 1.000000e-16};
      int derivLen = 33; //23; 13; 33;
      FLOAT_MAT_ELEM_TYPE smoothp[] = {-1.000000e-03, -2.200000e-02, -1.480000e-01, -1.940000e-01, 7.300000e-01, -1.940000e-01, -1.480000e-01, -2.200000e-02, -1.000000e-03};
      //{-1.000000e-07, -5.400000e-06, -1.240000e-04, -1.561000e-03, -1.149400e-02, -4.787020e-02, -9.073680e-02, 2.144300e-02, 2.606970e-01, 2.144300e-02, -9.073680e-02, -4.787020e-02, -1.149400e-02, -1.561000e-03, -1.240000e-04, -5.400000e-06, -1.000000e-07};
      int smoothLen = 9; //9; 17;
      //horizontal is derivative and vertical is smoothing
      fy = cvMat(1, smoothLen, FLOAT_MAT_TYPE, smoothp);
      fx = cvMat(derivLen, 1, FLOAT_MAT_TYPE, derivp);
    }
    break;
  }


#warning "still check subtracting mean from image"
  //subtract mean
  CvScalar mean = cvAvg(inImage);
  cvSubS(inImage, mean, outImage);


  //do the filtering
  cvFilter2D(outImage, outImage, &fx); //inImage outImage
  cvFilter2D(outImage, outImage, &fy);

}

/**
 * This function gets a 1-D gaussian filter with specified
 * std deviation and range
 *
 * \param kernel input mat to hold the kernel (2*w+1x1)
 *      column vector (already allocated)
 * \param w width of kernel is 2*w+1
 * \param sigma std deviation
 */

void callbackFunc(int event, int x, int y, int flags, void* userdata) {
    bool *done = (bool*)userdata;
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (no_clicks < 4) {
            src_vertices[no_clicks] = cv::Point(x, y);
            std::cout << "x: " << x << "y: " << y << std::endl;
            no_clicks++;
            if (no_clicks == 4) *done = true;
        } else {
            *done = true;
        }
    }
}



cv::Mat transformImage(cv::Mat &image) {
    cv::Point2f dst_vertices[4];



    dst_vertices[0] = cv::Point(0,0 );
    dst_vertices[1] = cv::Point(1000,0 );
    dst_vertices[2] = cv::Point(0,1000 );
    dst_vertices[3] = cv::Point(1000,1000 );
    cv::Mat wrap_perspective_transform = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::Mat result;
    cv::warpPerspective(image, result, wrap_perspective_transform, cv::Size(1000, 1000), cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    return result;
}

cv::Mat transformImage_back(cv::Mat &image) {

    cv::Point2f dst_vertices[4];

    dst_vertices[0] = cv::Point(0, 0);
    dst_vertices[1] = cv::Point(1000, 0);
    dst_vertices[2] = cv::Point(0, 1000);
    dst_vertices[3] = cv::Point(1000, 1000);
    cv::Mat wrap_perspective_transform = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    cv::Mat result;
    wrap_perspective_transform=wrap_perspective_transform.inv();
    cv::warpPerspective(image, result, wrap_perspective_transform, cv::Size(640,480), cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    return result;
}

void mcvLoadImage(const char *filename, CvMat **clrImage, CvMat** channelImage)
{
 /* // load the image
  IplImage* im;
  im = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
  // convert to mat and get first channel
  CvMat temp;
  cvGetMat(im, &temp);
  *clrImage = cvCloneMat(&temp);
  // convert to single channel
  CvMat *schannel_mat;
  CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
  cvSplit(*clrImage, tchannelImage, NULL, NULL, NULL);
  // convert to float
  *channelImage = cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
  cvConvertScale(tchannelImage, *channelImage, 1./255);
  // destroy
  cvReleaseMat(&tchannelImage);
  cvReleaseImage(&im);*/

// load the image
  IplImage* im;
  im = cvLoadImage(filename, CV_LOAD_IMAGE_COLOR);
  // convert to mat and get first channel
  CvMat temp;
  cvGetMat(im, &temp);
  *clrImage = cvCloneMat(&temp);
  // convert to single channel
  CvMat *schannel_mat;
  CvMat* tchannelImage = cvCreateMat(im->height, im->width, INT_MAT_TYPE);
  cvSplit(*clrImage, tchannelImage, NULL, NULL, NULL);
  // convert to float
  *channelImage = cvCreateMat(im->height, im->width, FLOAT_MAT_TYPE);
  cvConvertScale(tchannelImage, *channelImage, 1./255);
  // destroy
  cvReleaseMat(&tchannelImage);
  cvReleaseImage(&im);
}


void SHOW_IMAGE(const CvMat *pmat, const char str[], int wait)
{
  //cout << "channels:" << CV_MAT_CN(pmat->type) << "\n";
  //scale it
  //CvMat *mat = cvCreateMat(pmat->height, pmat->width, pmat->type);
  //cvCopy(pmat, mat);
  CvMat *mat = cvCloneMat(pmat);//->rows, pmat->cols, INT_MAT_TYPE);//cvCloneMat(pmat);
  assert(mat);
  //convert to int type
 
  //show it
  //cout << "in\n";
  cvNamedWindow(str, CV_WINDOW_AUTOSIZE); //0 1
  cvShowImage(str, mat);
  cvWaitKey(wait);
  //cvDestroyWindow(str);
  //clear
  cvReleaseMat(&mat);
  //cout << "out\n";
}


int main()
{
	//initialize important variables here
	int n_segments=5;
	int segments[5]={75, 140, 215, 250, 320};
	Mat img_segments[5];
	std::stringstream window_name;
      cv::namedWindow("Original Image");
      Mat pimage=imread("test.png",1);

      cv::setMouseCallback("Original Image", callbackFunc, &done);
      imshow("Original Image",pimage);
     
      waitKey(10000); 

		  VideoCapture cap("../aa.mp4"); // open the default camera
      if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat edges;
    namedWindow("edges",1);



    while(1)
    {

        Mat frame;
        cap >> frame;
        

        Mat img=frame; 
       cvtColor(frame,frame,COLOR_BGR2GRAY);
       //transform+filter+inverse transform
        imshow("transform",transformImage(frame));
        Mat transform=transformImage(frame);
        CvMat * ipm,*ipm2;

        imwrite("test2.png",transform);
        mcvLoadImage("test2.png", &ipm, &ipm2);
        //Values set approximately resulting in not so clear filtering
        mcvFilterLines(ipm, ipm, 2,2, 2000, 304.8,LINE_VERTICAL);
        
        SHOW_IMAGE(ipm, "filter", 5);
        
        //inverse tranform me load ho raha check it
        Mat invtransform=transformImage_back(frame);


        imshow("transformImage",transformImage_back(frame));

        //transform(frame);


        //--------------------------------------
    //cv::Mat img=cv::imread("/home/harshit/bsnake_lane_detection/test.jpg", CV_LOAD_IMAGE_COLOR);
	cv::resize(img, img, cv::Size(1000,1000));
	imshow("lanes", img);

	int max_lowThreshold=500, max_highThreshold=500;
	/*namedWindow("Edge threshold", CV_WINDOW_AUTOSIZE);
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
*/
	edges=find_edges(img);
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
	int hough_threshold[5]={30, 30, 40, 50, 50};
	int hough_minLineLength[5]={20, 25, 25, 30, 50};
	for(i=0; i<n_segments ;i++)
		{
			HoughLinesP(img_segments[i], lines[i], 1, CV_PI/180, hough_threshold[i], hough_minLineLength[i], 50 );
         for(int j=0;j<lines[i].size();j++)
          {
            //take out unwanted near-horizontal lines that spoil the votes
            
             double m=((double)lines[i][j][3]-lines[i][j][1])/((double)lines[i][j][2]-lines[i][j][0]);
            if((m<0.4&&m>0)||(m>-0.4&&m<0))
            {
             lines[i][j][1]=lines[i][j][3];
             lines[i][j][0]=lines[i][j][2];
             }
          }
    }
  	Mat line_segments[n_segments];
  	Mat empty=img-img;
  	extract_segments(line_segments, empty, segments, n_segments);


  	for(i=0;i<n_segments;i++)
  		for(j=0;j<lines[i].size();j++)
  		{
  			Vec4i l = lines[i][j];
        cout<<lines[i][j][1]<<endl;
     
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

  	for(i=0;i<n_segments;i++)
  		cout<<"#lines for "<<i<<": "<<lines[i].size()<<endl;

  	int vanish_row_vote[2000]={0};

  	int cum_sum=1000;
  	for(i=4;i>=3;i--)
  	{
  		cum_sum-=segments[i];
  		for(j=0;j<lines[i].size();j++)
  			for(k=0;k<lines[i].size();k++)
  			{
  				if(j==k)
  					continue;

  				int vanish_row=find_intersection(lines[i][j], lines[i][k])+cum_sum;

  				//for checking intersection function
  				/*Mat ci(1000, 1000, CV_8UC3, Scalar(0));
  				Mat cs[5];
  				extract_segments(cs, ci, segments, n_segments);
  				cv::line( cs[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  				cv::line( cs[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  				merge_segments(cs, ci, segments, n_segments);
  				imshow("check", ci);

  				cout<<1000-vanish_row<<endl;
  				waitKey(2000);*/

  				if(vanish_row>-1000 && vanish_row<1000)
  					vanish_row_vote[1000-vanish_row]++;
  			}
  	}

  	int current_votes=0;
  	int max_votes=-1, max_i=-1;
  	for(i=0;i<50;i++)
  		current_votes+=vanish_row_vote[i];

  	for(i=50;i<2000;i++)
  	{
  		current_votes+=vanish_row_vote[i];
  		current_votes-=vanish_row_vote[i-50];

  		if(current_votes>=max_votes)
  		{
  			max_votes=current_votes;
  			max_i=i;
  		}

  		//cout<<i<<" "<<current_votes<<endl;
  	}

  	int vanish_row=max_i-25;

  	cout<<"Vanishing row: "<<vanish_row<<" with votes: "<<max_votes<<endl;

  	Mat output(1200, 1000, CV_8UC3, Scalar(0));
  	line.copyTo(output(cv::Rect(0, 200, 1000, 1000)));
  	cv::line( output, Point(0, 1200-vanish_row), Point(1000, 1200-vanish_row), Scalar(255,0,0), 10, CV_AA, 0);
  	//line( output, Point(0, 1000-vanish_row), Point(1000, 1000-vanish_row), Scalar(255,0,0), 10, CV_AA);


  	imshow("output", output);

  	Mat lanes(1000, 1000, CV_8UC3, Scalar(0));
  	Mat lanes_segments[n_segments];
  	extract_segments(lanes_segments, lanes, segments, n_segments);

  	cum_sum=1000;
  	for(i=4;i>=2;i--)
  	{
  		cum_sum-=segments[i];
  		for(j=0;j<lines[i].size();j++)
  			for(k=0;k<lines[i].size();k++)
  			{
  				if(j==k)
  					continue;

  				int vanishRow=find_intersection(lines[i][j], lines[i][k])+cum_sum;

  				if(1000-vanishRow>= vanish_row-20 && 1000-vanishRow<= vanish_row+20)
  				{
  					if(i==4)
  						//cout<<"yay"<<j<<endl;


  					cv::line( lanes_segments[i], Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( lanes_segments[i], Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					/*imshow("please", lanes_segments[i]);

  					Mat temp(segments[i], 1000, CV_8UC3, Scalar(0));
  					cv::line( temp, Point(lines[i][j][0], lines[i][j][1]), Point(lines[i][j][2], lines[i][j][3]), Scalar(255,0,0), 3, CV_AA, 0);
  					cv::line( temp, Point(lines[i][k][0], lines[i][k][1]), Point(lines[i][k][2], lines[i][k][3]), Scalar(255,0,0), 3, CV_AA, 0);

  					imshow("verify", temp);

  					waitKey(2000);*/

  				}	
  			}
  	}
  	merge_segments(lanes_segments, lanes, segments, n_segments);

  	for(i=1000-vanish_row;i>=0;i--)
  		for(j=0;j<img.cols;j++)
  			lanes.at<Vec3b>(i, j)={0, 0, 0};

  	for(i=0;i<img.rows;i++)
  		for(j=0;j<img.cols;j++)
  			if(lanes.at<Vec3b>(i, j)[0]==255)
  				img.at<Vec3b>(i, j)={255, 0, 0};


  	imshow("yay", img);
  	imshow("wohoo!", lanes);
    Size size(640,480);
    resize(lanes,lanes,size);
    imwrite("/home/harshit/caltech-lane-detection/src/output.png",lanes);

	imshow("edges", edges);
	waitKey(10);
	
}
	
}