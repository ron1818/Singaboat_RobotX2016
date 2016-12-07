/*This node funtions are:
  + Detect red, yellow, green, and blue tiki totem for task 2 (navigation)
  + Detect black buoys (which are obstacles)
  + Detect white totem marker (since this task's area is marked by white totem marker)
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <robotx_vision/object_detection.h>

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
std::string frame_id;
std::string output_topic_name;

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//Must-have var
vector<robotx_vision::object_detection> object;

//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
std::vector<cv::Point> approx;
cv::Mat src, hsv, hls;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat color, white;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Rect r;
cv::RotatedRect mr;
int height, width;
const int min_area = 400;
double area, r_area, m_area;
const double eps = 0.15;

//Functions
double v_angle(int y)
{
  //Camera's vertical angle of view (calculated) is 65.28
  return atan(tan(0.560)*(2*(double)y/height-1)); //Angle in rad
}

double h_angle(int x)
{
  //Camera's horizontal angle of view (as in specification) is 81
  return atan(tan(0.707)*(2*(double)x/width-1)); //Angle in rad
}

void angle(robotx_vision::object_detection* ob, cv::Rect rect)
{
  ob->angle_t = v_angle((rect.tl()).y);
  ob->angle_b = v_angle((rect.br()).y);
  ob->angle_l = h_angle((rect.tl()).x);
  ob->angle_r = h_angle((rect.br()).x);
}

void reduce_noise(cv::Mat* bgr)
{
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*bgr, *bgr, cv::MORPH_OPEN, str_el);
  cv::blur(*bgr,*bgr,Size(2,2));
}

robotx_vision::object_detection object_return(std::string frame_id, std::string type, std::string color, cv::Rect rect)
{
  robotx_vision::object_detection obj;
  obj.frame_id = frame_id;
  obj.type = type;
  obj.color = color;
  angle(&obj, rect);
  return obj;
}

void detect_tiki(std::string obj_color, cv::Scalar up_lim, cv::Scalar low_lim, cv::Scalar up_lim_wrap = cv::Scalar(0,0,0), cv::Scalar low_lim_wrap = cv::Scalar(0,0,0))
{
  //Filter desired color
  if(up_lim_wrap == low_lim_wrap)
    cv::inRange(hsv, up_lim, low_lim, color);
  else
  {//In case of red color
    cv::inRange(hsv, up_lim, low_lim, lower_hue_range);
    cv::inRange(hsv, up_lim_wrap, low_lim_wrap, upper_hue_range);
    cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,color);
  }
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

    r = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    r_area = r.height*r.width;
    m_area = (mr.size).height*(mr.size).width;
    //Detect
    if((fabs(r.height/r.width - 2) < (0.2+eps)) && (area/m_area > (0.95-eps)))
    { //If a tiki totem is detected
      cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 8, 8, 0);
      object.push_back(object_return(frame_id,"tiki_totem",obj_color,r));         //Push the object to the vector
    }
  }
}

void detect_obstacle()
{
  //Filter black color
  cv::inRange(hsv, cv::Scalar(117, 40, 4), cv::Scalar(140, 255, 150), color);
  //Reduce noise
  reduce_noise(&color);
  //Finding shapes
  cv::findContours(color.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accura cy proportional to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
    // Skip small or non-convex objects 
    if ((std::fabs(cv::contourArea(contours[i])) < min_area) /*|| (!cv::isContourConvex(approx))*/) continue;
    //Find black buoys
    area = cv::contourArea(contours[i]);
    r = cv::boundingRect(contours[i]);
    if(r.height > r.width) continue;
    r_area = r.height*r.width;
    if((std::abs(area/r_area - 3.141593/4) <= 0.1) && (std::abs((double)r.height/r.width - 0.833) < 0.2))
    {//Detected
      cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
      object.push_back(object_return(frame_id,"obstacle","black",r));   //Push the object to the vector
    }
  }
}

void detect_white_marker()
{
  //Filter red color
  cv::inRange(hls, cv::Scalar(5, 225, 2), cv::Scalar(255, 255, 255), white);
  //Reduce noise
  reduce_noise(&white);
  //Finding shapes
  cv::findContours(white.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;
    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    r = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    double area = cv::contourArea(contours[i]);
    double r_area = (mr.size).height*(mr.size).width;
    double hull_area = contourArea(hull);

    if((fabs((double)r.height/r.width - 1.9) < (0.2+eps)) && (fabs((double)area/r_area - 0.65) < eps) && (fabs(((double)area/hull_area - 0.85) < eps)) || (fabs((double)r.height/r.width - 1.9) < (0.2+eps)) && ((area/r_area - 0.8) < eps))
    { //If a white totem is detected
      cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 8, 8, 0);
      object.push_back(object_return(frame_id,"marker_totem","white",r));         //Push the object to the vector
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
  detect_white_marker();
  detect_tiki("blue", cv::Scalar(105, 100, 100), cv::Scalar(130, 255, 255));
  detect_tiki("green", cv::Scalar(50, 130, 70), cv::Scalar(96, 255, 255));
  detect_tiki("yellow", cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255));
  detect_tiki("red", cv::Scalar(0, 80, 100), cv::Scalar(1, 255, 255), cv::Scalar(165, 80, 100), cv::Scalar(179, 255, 255));
  cv::imshow("color",color);
  detect_obstacle();
  //Show output on screen
  //ROS_INFO("Node is working.");
  cv::imshow("src", src);
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "obstacle_tiki_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("frame_id", frame_id);
  pnh.getParam("output_topic_name", output_topic_name);

  //Initiate windows
  cv::namedWindow("src",WINDOW_NORMAL);
  cv::resizeWindow("src",640,480);
  cv::namedWindow("color",WINDOW_NORMAL);
  cv::startWindowThread();

  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);

  //...and ROS publisher
  ros::Publisher pub = nh.advertise<robotx_vision::object_detection>(output_topic_name, 1000);
  ros::Rate r(30);

  while (nh.ok())
  {
  	//Publish every object detected
    for(vector<robotx_vision::object_detection>::iterator it = object.begin(); it != object.end(); it++)
      pub.publish(*it);
    //Reinitialize the object counting vars
    object.clear();

    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}