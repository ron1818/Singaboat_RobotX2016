/*This node funtion(s):
  + Detect symbols: red/green/blue circle/triangle/cruciform
  + Detect markers: red/green/blue/yellow/white (totem) markers
  + Detect obstacles
  + And also give the distance to the object using zed camera
*/
//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <dynamic_reconfigure/server.h>
#include <robotx_vision/detectionConfig.h>
#include <std_msgs/Float64.h>
//OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//C++ standard libs
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
//Namespaces
using namespace ros;
using namespace cv;
using namespace std;
//ROS params
std::string subscribed_image_topic, subscribed_depth_topic;
std::string object_shape, object_color;
std::string published_roi_topic, published_distance_topic;
double far_dist = 10;
double near_dist = 0.5;
bool debug;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr, depth_ptr;
//ROS var
vector<sensor_msgs::RegionOfInterest> object;
vector<std_msgs::Float64> dist;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat src, hsv, hls, depth_mat;
cv::Scalar up_lim, low_lim, up_lim_wrap, low_lim_wrap;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat color;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
int min_area = 300;
double area, r_area, m_area;
const double eps = 0.15;
//Functions
void reduce_noise(cv::Mat* dst)
{
  cv::morphologyEx(*dst, *dst, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*dst, *dst, cv::MORPH_OPEN, str_el);
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

void object_found()
{
  object.push_back(object_return());         //Push the object to the vector
  if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
}

void detect_marker()
{
  //Filter desired color
  if(object_color == "red")
  {//In case of red color
    cv::inRange(hsv, low_lim, up_lim, lower_hue_range);
    cv::inRange(hsv, low_lim_wrap, up_lim_wrap, upper_hue_range);
    cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,color);
  }
  else if(object_color == "white")
  {
    cv::cvtColor(src,hsv,COLOR_BGR2HLS); //hsv is in HSL image type
    cv::inRange(hsv, low_lim, up_lim, color);
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
    //cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.01, true);
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    r_area = rect.height*rect.width;
    m_area = (mr.size).height*(mr.size).width;
    //Detect
    if(object_color != "red")
      if((rect.height/rect.width > 1) && (area/m_area > (0.95-eps)))
        object_found();
    else 
    {
      vector<Point> hull;
      convexHull(contours[i], hull, 0, 1);
      double hull_area = contourArea(hull);
      if(((rect.height/rect.width > 1)&&(fabs(area/r_area - 0.65) < eps)&&(fabs(area/hull_area - 0.85) < eps)) 
              || ((rect.height/rect.width > 1) && (fabs(area/r_area - 0.8) < eps)))
        object_found();
    }
  }
}

void detect_obstacle()
{
  //Filter black color
  cv::inRange(hsv, low_lim, up_lim, color);
  //Reduce noise
  reduce_noise(&color);
  //Finding shapes
  cv::findContours(color.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accura cy proportional to the contour perimeter
    //cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
    // Skip small or non-convex objects 
    if ((std::fabs(cv::contourArea(contours[i])) < min_area) /*|| (!cv::isContourConvex(approx))*/) continue;
    //Find black buoys
    rect = cv::boundingRect(contours[i]);
    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);

    area = cv::contourArea(contours[i]);
    r_area = rect.height*rect.width;
    double hull_area = contourArea(hull);

    if((rect.height > rect.width) || (fabs(area/hull_area - 1) < eps)) continue; //Skip false objects
    if((std::abs(area/r_area - 3.141593/4) <= 0.1) && (std::abs((double)rect.height/rect.width - 0.833) < 0.2))
      object_found();
  }
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
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    m_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    double hull_area = contourArea(hull);

    if(object_shape == "circle") 
    {
      if((std::fabs(area/m_area - 3.141593/4) < 0.1) && (std::fabs(area/hull_area - 1) < 0.05))
        object_found();
    }
    else if(object_shape == "triangle") 
    {
      if((fabs(area/m_area - 0.5) < 0.07 && (std::fabs(area/hull_area - 1) < 0.05)) /*&& cv::isContourConvex(approx)*/)
        object_found();
    }
    else if(object_shape == "cruciform")
    {
      Point2f center(contours[i].size());
      float radius(contours[i].size());
      Point2f hull_center(hull.size());
      float hull_radius(hull.size());
      minEnclosingCircle(contours[i], center, radius);
      minEnclosingCircle(hull, hull_center, hull_radius);
      double cir_area = 3.1416*radius*radius;
      double hull_cir_area = 3.1416*hull_radius*hull_radius;
      double eps = (2 - fabs(area/cir_area - 0.5) - fabs(hull_area/hull_cir_area - 0.8))/2-1;
      if(fabs(eps) <= 0.1)
        object_found();
    }
  }
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //Get the image in OpenCV format
  src = cv_ptr->image;
  if (src.empty()) return;
  //Start the shape detection code
  cv::blur(src,src,Size(1,1));
  cv::cvtColor(src,hsv,COLOR_BGR2HSV);
  width = src.cols;
  height = src.rows;
  //Detect stuffs
  if(object_shape == "marker" || object_shape == "totem") detect_marker();
  else if(object_shape == "obstacle") detect_obstacle();
  else detect_symbol();
  //Show output on screen in debug mode
  if(debug) 
  {
    cv::imshow("src", src);
    cv::imshow("color", color);
  }
  //Find distance to every detected object
  //by getting the avarage depth value to the ROI
  for(vector<sensor_msgs::RegionOfInterest>::iterator it = object.begin(); it != object.end(); it++)
  {
    int n = 0;
    double sum = 0;
    for (int x = it->x_offset; x <= (it->x_offset + it->width); x++)
      for (int y = it->y_offset; y <= (it->y_offset + it->height); y++)
        {
          cv::Scalar intensity = depth_mat.at<float>(y, x); 
          double pixel_depth = intensity.val[0];
          if(!isnan(pixel_depth) && (pixel_depth > near_dist) && (pixel_depth < far_dist))
          {
            n++;
            sum += pixel_depth;
          }
        }
    std_msgs::Float64 distance_msg;
    distance_msg.data = sum/n;
    dist.push_back(distance_msg);
  }
}

void depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
{
	try
	{
		depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
	}
	catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //refresh depth image
  depth_mat = depth_ptr->image;
}

void dynamic_configCb(robotx_vision::detectionConfig &config, uint32_t level) 
{
  min_area = config.min_area;
  //Process appropriate parameter for object shape/color
  if(object_color == "blue") 
  {
    low_lim = cv::Scalar(config.blue_H_low,config.blue_S_low,config.blue_V_low);
    up_lim = cv::Scalar(config.blue_H_high,config.blue_S_high,config.blue_V_high);
  }
  else if(object_color == "green")
  {
    low_lim = cv::Scalar(config.green_H_low,config.green_S_low,config.green_V_low);
    up_lim = cv::Scalar(config.green_H_high,config.green_S_high,config.green_V_high);
  }
  else if(object_color == "yellow")
  {
    low_lim = cv::Scalar(config.yellow_H_low,config.yellow_S_low,config.yellow_V_low);
    up_lim = cv::Scalar(config.yellow_H_high,config.yellow_S_high,config.yellow_V_high);
  }
  else if(object_color == "red")
  {
    low_lim = cv::Scalar(config.red_H_low1, config.red_S_low, config.red_V_low);
    up_lim = cv::Scalar(config.red_H_high1, config.red_S_high, config.red_V_high);
    low_lim_wrap = cv::Scalar(config.red_H_low2, config.red_S_low, config.red_V_low);
    up_lim_wrap = cv::Scalar(config.red_H_high2, config.red_S_high, config.red_V_high);
  }
  else if(object_color == "white")
  {
    low_lim = cv::Scalar(config.white_H_low,config.white_S_low,config.white_V_low);
    up_lim = cv::Scalar(config.white_H_high,config.white_S_high,config.white_V_high);
  }
  else if(object_shape == "obstacle") //obstacles
  {
    low_lim = cv::Scalar(config.black_H_low,config.black_S_low,config.black_V_low);
    up_lim = cv::Scalar(config.black_H_high,config.black_S_high,config.black_V_high);
  }
  
  ROS_INFO("Reconfigure Requested.");
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "depth_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("subscribed_depth_topic", subscribed_depth_topic);
  pnh.getParam("object_shape", object_shape);
  pnh.getParam("object_color", object_color);
  pnh.getParam("published_roi_topic", published_roi_topic);
  pnh.getParam("published_distance_topic", published_distance_topic);
  pnh.getParam("debug", debug);
  //Dynamic reconfigure
  dynamic_reconfigure::Server<robotx_vision::detectionConfig> server;
  dynamic_reconfigure::Server<robotx_vision::detectionConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);
  //Initiate windows
  if(debug)
  { 
    cv::namedWindow("color",WINDOW_NORMAL);
    cv::resizeWindow("color",640,480);
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::resizeWindow("src",640,480);
    cv::startWindowThread();
  }
  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe(subscribed_image_topic, 1, imageCb);
  image_transport::Subscriber depth_sub = it.subscribe(subscribed_depth_topic, 1, depthCb);
  //...and ROS publisher
  ros::Publisher roi_pub = nh.advertise<sensor_msgs::RegionOfInterest>(published_roi_topic, 1000);
  ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>(published_distance_topic, 1000);
  ros::Rate r(30);
  while (nh.ok())
  {
  	//Do callbacks
  	ros::spinOnce();
    //Publish every object detected
    if(object.size() == dist.size())
    	for(int i = 0; i < object.size(); i++)
	    {
	      roi_pub.publish(object[i]);
	      dist_pub.publish(dist[i]);
	    }
	 	else ROS_WARN("Unmatched numbers of objects & distances!");
    //Reinitialize the object counting vars
    object.clear();
    dist.clear();
    //Loop
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}