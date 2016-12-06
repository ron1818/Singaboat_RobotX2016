/*This node funtion(s):
	+ Detect symbols: red/green/blue circle/triangle/cruciform
*/

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
#include <vector>
#include <cmath>


//Namespaces
using namespace ros;
using namespace cv;
using namespace std;

//ROS params
std::string subscribed_image_topic;
std::string object_shape, object_color;
std::string published_topic;
bool debug;

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//ROS var
vector<sensor_msgs::RegionOfInterest> object;

//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Mat src, hsv, hls;
cv::Scalar up_lim, low_lim, up_lim_wrap, low_lim_wrap;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat color;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
int min_area = 500;
double area, r_area, m_area;
const double eps = 0.15;

//Functions
void reduce_noise(cv::Mat* bgr)
{
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_OPEN, str_el);
  cv::blur(*bgr,*bgr,Size(2,2));
}

sensor_msgs::RegionOfInterest object_return()
{
  sensor_msgs::RegionOfInterest obj;
  obj.x_offset = (rect.tl()).x;
  obj.y_offset = (rect.tl()).y;
  obj.height = rect.height;
  obj.width = rect.width;
  obj.do_rectify = true;
  return obj;
}

int tri_detect(int i)
{
  if((fabs(area/m_area-0.5)<0.08) /*&& cv::isContourConvex(approx)*/)
  {
    object.push_back(object_return());         //Push the object to the vector
    if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
    return 1;
  }
  return 0;
}

int cir_detect(int i)
{
  if((std::abs(area/m_area - 3.141593/4) < 0.12) && cv::isContourConvex(approx))
  {
    object.push_back(object_return());         //Push the object to the vector
    if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
    return 1;
  }
  return 0;
}

int cru_detect(int i)
{
  vector<Point> hull;
  Point2f center(contours[i].size());
  float radius(contours[i].size());
  convexHull(contours[i], hull, 0, 1);
  Point2f hull_center(hull.size());
  float hull_radius(hull.size());
  double hull_area = contourArea(hull);
  minEnclosingCircle(contours[i], center, radius);
  minEnclosingCircle(hull, hull_center, hull_radius);
  double cir_area = 3.1416*radius*radius;
  double hull_cir_area = 3.1416*hull_radius*hull_radius;
  double eps = (2 - fabs(area/cir_area - 0.5) - fabs(hull_area/hull_cir_area - 0.8))/2-1;
  if(fabs(eps)<=0.1)
  {
    object.push_back(object_return());         //Push the object to the vector
    if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
    return 1;
  }
  return 0;
}

void detect_symbol()
{
  //Filter desired color
  if(object_color == "red")
  {//In case of red color
    cv::inRange(hsv, low_lim, up_lim, lower_hue_range);
    cv::inRange(hsv, low_lim_wrap, up_lim_wrap, upper_hue_range);
    cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,color);
  }
  else cv::inRange(hsv, low_lim, up_lim, color);
  //Reduce noise
  reduce_noise(&color);
  //Finding shapes
  cv::findContours(color.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accuracy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    r_area = rect.height*rect.width;
    m_area = (mr.size).height*(mr.size).width;

    if(object_shape == "circle") 
      if((std::abs(area/m_area - 3.141593/4) < 0.1) && cv::isContourConvex(approx))
      {
        object.push_back(object_return());         //Push the object to the vector
        if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
      }
    else if(object_shape == "triangle") 
      if((fabs(area/m_area - 0.5) < 0.07) /*&& cv::isContourConvex(approx)*/)
      {
        object.push_back(object_return());         //Push the object to the vector
        if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
      }   
    else if(object_shape == "cruciform")
      {
        vector<Point> hull;
        Point2f center(contours[i].size());
        float radius(contours[i].size());
        convexHull(contours[i], hull, 0, 1);
        Point2f hull_center(hull.size());
        float hull_radius(hull.size());
        double hull_area = contourArea(hull);
        minEnclosingCircle(contours[i], center, radius);
        minEnclosingCircle(hull, hull_center, hull_radius);
        double cir_area = 3.1416*radius*radius;
        double hull_cir_area = 3.1416*hull_radius*hull_radius;
        double eps = (2 - fabs(area/cir_area - 0.5) - fabs(hull_area/hull_cir_area - 0.8))/2-1;
        if(fabs(eps) <= 0.1)
        {
          object.push_back(object_return());         //Push the object to the vector
          if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
        }
      }
  }
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
  //newImage = true;
  //Start the shape detection code
  if (src.empty()) return;
  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
  cv::cvtColor(src,hls,COLOR_BGR2HLS);
  width = src.cols;
  height = src.rows;
  //Detect stuffs
  if(object_shape == "marker") detect_marker();
  else detect_symbol();
  //Show output on screen in debug mode
  if(debug) 
  {
    cv::imshow("src", src);
    cv::imshow("color", color);
  }
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "vision");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("object_shape", object_shape);
  pnh.getParam("object_color", object_color);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  //Process appropriate parameter for object shape/color
  if(object_color == "blue") 
  {
  	low_lim = cv::Scalar(105,100,100);
    up_lim = cv::Scalar(130,255,255);
  }
  else if(object_color == "green")
  {
    low_lim = cv::Scalar(50,55,55);
    up_lim = cv::Scalar(96,150,150);
  }
  else if(object_color == "red")
  {
    low_lim = cv::Scalar(0,80,100);
    up_lim = cv::Scalar(5,255,255);
    low_lim_wrap = cv::Scalar(165,80,100);
    up_lim_wrap = cv::Scalar(179,255,255);
  }
  //Initiate windows
  if(debug)
  {
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::resizeWindow("src",640,480);
    cv::namedWindow("color",WINDOW_NORMAL);
    cv::resizeWindow("color",640,480);
    cv::startWindowThread();
  }
  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);
  //...and ROS publisher
  ros::Publisher pub = nh.advertise<sensor_msgs::RegionOfInterest>(published_topic, 1000);
  ros::Rate r(30);
  while (nh.ok())
  {
  	//Publish every object detected
    for(vector<sensor_msgs::RegionOfInterest>::iterator it = object.begin(); it != object.end(); it++)
      pub.publish(*it);
    //Reinitialize the object counting vars
    object.clear();

    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}