//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

//OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//C++ standard libs
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>


//Namespaces
using namespace ros;
using namespace cv;
using namespace std;

std::string nodeName = "black_buoy";
std::string subscribedTopic = "/port/right/image_rect_color";

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//Image processing vars
bool newImage = false;

//OpenCV image vars
cv::Mat src, hsv;
cv::Mat black;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));

//OpenCV shape detection vars
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Rect r;
double area, r_area;

void reduce_noise(cv::Mat* bgr)
{
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_OPEN, str_el);
  cv::blur(*bgr,*bgr,Size(2,2));
}

int buoy_detect(Scalar color, int i)
{
	if((std::abs(area/r_area - 3.141593/4) <= 0.1))
    {
  		cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), color, 2, 8, 0);
  		return 1;
    }
  return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //Get the image in OpenCV format
  src = cv_ptr->image;
  newImage = true;

  //Start the shape detection code
  if (src.empty()) return;

  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
	//Filter black color
	cv::inRange(hsv, cv::Scalar(110, 70, 5), cv::Scalar(150, 255, 100), black);

	//Reduce noise
	reduce_noise(&black);

	//Finding shapes
	cv::findContours(black.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

	//Detect shape for each contour
	for (int i = 0; i < contours.size(); i++)
	{
		// Approximate contour with accura cy proportional to the contour perimeter
		cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

		// Skip small or non-convex objects 
		if ((std::fabs(cv::contourArea(contours[i])) < 800) || (!cv::isContourConvex(approx))) continue;

		area = cv::contourArea(contours[i]);
		r = cv::boundingRect(contours[i]);
		if(r.height > r.width) continue;
		r_area = r.height*r.width;

		buoy_detect(Scalar(0,255,255), i);
	}
  

  cv::imshow("black",black);
  cv::imshow("src", src);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;

    //Initiate windows
    cv::namedWindow("src",CV_WINDOW_NORMAL);
    cv::namedWindow("black",CV_WINDOW_NORMAL);
    cv::startWindowThread();

    //Start ROS
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(subscribedTopic, 1, imageCb);
    ros::Rate r(40);

    while (nh.ok())
    {
        /*if (newImage)
        {
            imageProcess();
        }*/
        ros::spinOnce();
        r.sleep();
    }
    cv::destroyAllWindows();
    return 0;
}
