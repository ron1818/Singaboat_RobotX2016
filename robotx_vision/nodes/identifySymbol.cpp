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
#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>


//Namespaces
using namespace ros;
using namespace cv;
using namespace std;

std::string nodeName = "shape_detection";

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//Image processing vars
bool newImage = false;

//OpenCV image vars
cv::Mat src, hsv;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat red,green,blue;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));

//OpenCV shape detection vars
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Rect r;
double area, r_area;

//Environmental param vars
//int green_min=35, green_max=75;

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 10);
}

void reduce_noise(cv::Mat* bgr)
{
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_OPEN, str_el);
  cv::blur(*bgr,*bgr,Size(2,2));
}

int tri_detect(Scalar color, int i)
{
  if((fabs(area/r_area-0.5)<=0.08) && cv::isContourConvex(approx))
  {
    cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), color, 2, 8, 0);
    setLabel(src, "TRI", contours[i]);
    return 1;
  }
  return 0;
}

int cir_detect(Scalar color, int i)
{
  if((std::abs(area/r_area - 3.141593/4) <= 0.16) && cv::isContourConvex(approx))
    {
    cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), color, 2, 8, 0);
    setLabel(src, "CIR", contours[i]); 
    return 1;
    }
    return 0;
}

int cru_detect(Scalar color, int i)
{
  vector<Point> hull;
  Point2f center(contours[i].size());
  float radius(contours[i].size());
  convexHull(contours[i], hull, 0, 1);
  Point2f hull_center(hull.size());
  float hull_radius(hull.size());
  double area = contourArea(contours[i]);
  double hull_area = contourArea(hull);
  minEnclosingCircle(contours[i], center, radius);
  minEnclosingCircle(hull, hull_center, hull_radius);
  double cir_area = 3.1416*radius*radius;
  double hull_cir_area = 3.1416*hull_radius*hull_radius;
  double eps = (2 - fabs(area/cir_area - 0.5) - fabs(hull_area/hull_cir_area - 0.8))/2-1;
  if(fabs(eps)<=0.1)
  {
    cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), color, 2, 8, 0);
    setLabel(src, "CRUCI", contours[i]);
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

  //Create window
  cv::namedWindow("src", WINDOW_AUTOSIZE);

  //Filter red color
  cv::inRange(hsv, cv::Scalar(0, 80, 100), cv::Scalar(10, 255, 255), lower_hue_range);
  cv::inRange(hsv, cv::Scalar(160, 80, 100), cv::Scalar(179, 255, 255), upper_hue_range);
  cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,red);

  //Reduce noise
  reduce_noise(&red);

  //Finding shapes
  cv::findContours(red.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);

    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < 2000) continue;

    area = cv::contourArea(contours[i]);
    r = cv::boundingRect(contours[i]);
    r_area = r.height*r.width;

    if(tri_detect(Scalar(0,0,255), i)) {}
    else if(cir_detect(Scalar(0,0,255), i))  {}
    else cru_detect(Scalar(0,0,255), i);
  }
  ///////////////////////////////////////////////////
  //Filter blue color
  cv::inRange(hsv, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), blue);

  //Reduce noise
  reduce_noise(&blue);

  //Finding shapes
  cv::findContours(blue.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);

    // Skip small or non-convex objects 
    if (std::fabs(cv::contourArea(contours[i])) < 2000)
      continue;

    area = cv::contourArea(contours[i]);
    r = cv::boundingRect(contours[i]);
    r_area = r.height*r.width;

    if(tri_detect(Scalar(255,0,0), i)) {}
    else if(cir_detect(Scalar(255,0,0), i)) {}
    else cru_detect(Scalar(255,0,0), i);
  }
  ///////////////////////////////////////////////////
  //Filter green color
  cv::inRange(hsv, cv::Scalar(70, 100, 70), cv::Scalar(100, 255, 255), green);

  //Reduce noise
  reduce_noise(&green);

  //Finding shapes
  cv::findContours(green.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    //Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);

    //Skip small or non-convex objects 
    if (std::fabs(cv::contourArea(contours[i])) < 2000)
      continue;

    area = cv::contourArea(contours[i]);
    r = cv::boundingRect(contours[i]);
    r_area = r.height*r.width;

    if(tri_detect(Scalar(0,255,0), i)) {}
    else if(cir_detect(Scalar(0,255,0), i)) {}
    else cru_detect(Scalar(0,255,0), i);
  }

  cv::imshow("src", src);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;

    //Initiate windows
    cv::namedWindow("src",CV_WINDOW_NORMAL);
    cv::namedWindow("green",CV_WINDOW_NORMAL);
    cv::startWindowThread();
    //cv::createTrackbar("min green hue", "green", &green_min, 179);
    //cv::createTrackbar("max green hue", "green", &green_max, 179);

    //Start ROS
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/port/right/image_rect_color", 1, imageCb);
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
