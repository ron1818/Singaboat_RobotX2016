/*This node funtion(s):
  + Detect symbols: red/green/blue circle/triangle/cruciform
  + Detect markers: red/green/blue/yellow/white (totem) markers
  + Detect obstacles
  + And also give the distance to the object using zed camera
  + Publish marker array, with the addition of base_link and camera_link
*/
//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include <robotx_vision/detectionConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
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
std::string published_topic = "/filtered_marker_array";
double far_dist = 20; //in meters
double near_dist = 0.5;
bool debug;
int object_type;
int object_id;
tf::TransformListener *tf_listener;
std::string camera_link = "/camera_link";
std::string base_link = "/base_link";
//default quaternion is 0:
geometry_msgs::Quaternion def_orientation;
//Image transport vars
cv_bridge::CvImagePtr cv_ptr, depth_ptr;
//ROS to-be-published var
visualization_msgs::MarkerArray object;
//Dynamic reconfigure parameters
int min_area = 300;
std::string show_screen;
cv::Scalar blue_low_lim, blue_up_lim;
cv::Scalar green_low_lim, green_up_lim;
cv::Scalar yellow_low_lim, yellow_up_lim;
cv::Scalar red_low_lim, red_up_lim, red_low_lim_wrap, red_up_lim_wrap;
cv::Scalar black_low_lim, black_up_lim;
cv::Scalar white_low_lim, white_up_lim;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat src, hsv, hls, depth_mat;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat color;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4,4));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
double area, r_area, mr_area, hull_area;
const double eps = 0.15;
//Functions
void reduce_noise(cv::Mat* dst)
{
  cv::morphologyEx(*dst, *dst, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(*dst, *dst, cv::MORPH_OPEN, str_el);
}

visualization_msgs::Marker object_return()
{
  visualization_msgs::Marker obj_holder;
  obj_holder.type = object_type;
  obj_holder.id = object_id;
  //set RGBAColor to yellow, rviz, not too important
  obj_holder.color.r = 255;
  obj_holder.color.g = 255;
  obj_holder.color.b = 0;
  obj_holder.color.a = 1;
  //Calculate pose in zed_camera frame
  	//orientation
  obj_holder.pose.orientation = def_orientation;
  	//position
  double object_angle = -atan(tan(0.95993) * ((double)2*rect.tl().x + rect.width - width) / width); //angle of zed camera is 110 -> 55 deg -> 0.95993 rad
  int n = 0;
  double sum = 0;
  for (int x = rect.tl().x; x < (rect.tl().x + rect.width); x++)
    for (int y = rect.tl().y; y < (rect.tl().y + rect.height); y++)
      {
        double pixel_depth = depth_mat.at<float>(y, x); 
        if(!isnan(pixel_depth) && (pixel_depth > near_dist) && (pixel_depth < far_dist))
        {
          n++;
          sum += pixel_depth;
        }
      }
  double distance = sum/n;
  //Transform data from zed_camera frame into base frame (world)
  tf::StampedTransform tf_transform;
  tf_listener->lookupTransform(base_link, camera_link, ros::Time(0), tf_transform);
  double del_x = tf_transform.getOrigin().x();
  double del_y = tf_transform.getOrigin().y();
  obj_holder.pose.position.x = del_x + distance * cos(object_angle);
  obj_holder.pose.position.y = del_y + distance * sin(object_angle);
  obj_holder.pose.position.z = 0;
  return obj_holder;
}

void object_found()
{
     //type:
  if(object_shape == "triangle")        object_type = 0;
  else if(object_shape == "cruciform")  object_type = 1;
  else if(object_shape == "circle")     object_type = 2;
  else if(object_shape == "totem")      object_type = 3;
  else if(object_shape == "rectangle")  object_type = 4;
  else if(object_shape == "pipe")       
  {
    object_type = 5;
    object_color = "white";
  }
    //id:
  if(object_color == "red")         object_id = 0;
  else if(object_color == "green")  object_id = 1;
  else if(object_color == "blue")   object_id = 2;
  else if(object_color == "black")  object_id = 3;
  else if(object_color == "white")  object_id = 4;
  else if(object_color == "yellow") object_id = 5;
  else if(object_color == "orange") object_id = 6;
  //Do pushback
  object.markers.push_back(object_return());         //Push the object to the vector
  if(debug) cv::rectangle(src, rect.tl(), rect.br()-cv::Point(1,1), cv::Scalar(0,255,255), 2, 8, 0);
}

void totem_detect(int i, std::string obj_color)
{
  object_shape = "totem";
  if(object_color == "red")
    if((((double)rect.height/rect.width > 1.2) && ((double)area/hull_area > (0.95-eps)))
            || (((double)rect.height/rect.width > 1.4) && (fabs((double)area/mr_area - 1) < eps)))
      object_found();
  else 
  {
    /*ROS_INFO("Object %d", i);
    cout << "H/W = " << (float)rect.height/rect.width << endl;
    cout << "area/min_rect_area = " << (float)area/mr_area << endl;
    cout << "area/hull_area = " << (float)area/hull_area << endl;
    cout << "hull_area/mr_area = " << (float)hull_area/mr_area << endl;
    cout << "H/W = " << (bool)((float)rect.height/rect.width > 1.2) << endl;
    cout << "area/min_rect_area = " << (bool)(fabs(area/mr_area - 0.5) < eps) << endl;
    cout << "area/hull_area = " << (bool)(fabs(area/hull_area - 0.68) < eps) << endl;
    cout << "hull_area/mr_area = " << (bool)(fabs(hull_area/mr_area - 0.72) < eps) << endl;*/
    if((((double)rect.height/rect.width > 1.4) && (fabs(area/mr_area - 0.5) < eps) && (fabs(area/hull_area - 0.68) < eps) && (fabs(hull_area/mr_area - 0.72) < eps)) 
            || (((double)rect.height/rect.width > 1.4) && (fabs(area/mr_area - 1) < eps)))
      object_found();
  }
}

void obstacle_detect(int i, std::string obj_color)
{
  object_shape = "obstacle";

  if((rect.height > rect.width) || (fabs(area/hull_area - 1) < eps)) return; //Skip false objects
  if((std::abs(area/r_area - 3.141593/4) <= 0.1) && (std::abs((double)rect.height/rect.width - 0.833) < 0.2))
    object_found();
}

void pipe_detect(int i, std::string obj_color)
{
  object_shape = "pipe";
  if(((double)rect.width/rect.height > 2) && (fabs(area/mr_area - 1) < eps))
    object_found();
}

int tri_detect(int i, std::string obj_color)
{
  object_shape = "triangle";
  if((std::fabs(area/mr_area - 3.141593/4) < 0.1) && (std::fabs(area/hull_area - 1) < 0.05))
    object_found();
  return 0;
}

int cir_detect(int i, std::string obj_color)
{
  object_shape = "circle";
  if((fabs(area/mr_area - 0.5) < 0.07 && (std::fabs(area/hull_area - 1) < 0.05)) /*&& cv::isContourConvex(approx)*/)
    object_found();
  return 0;
}

int cru_detect(int i, std::string obj_color)
{
  object_shape = "cruciform";
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
    object_found();
  return 0;
}

void do_inrange(std::string obj_color, cv::Scalar up_lim, cv::Scalar low_lim, cv::Scalar up_lim_wrap = cv::Scalar(0,0,0), cv::Scalar low_lim_wrap = cv::Scalar(0,0,0))
{
  //Filter desired color
  if(object_color == "white")
    cv::inRange(hls, up_lim, low_lim, color);
  else if(up_lim_wrap == low_lim_wrap)
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
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    rect = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    area = cv::contourArea(contours[i]);
    r_area = rect.height*rect.width;
    mr_area = (mr.size).height*(mr.size).width;

    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    hull_area = contourArea(hull);

    if(object_color == "red" || object_color == "blue" || object_color == "green")
    {
      tri_detect(i, obj_color);
      cir_detect(i, obj_color);
      cru_detect(i, obj_color);
    }
    if(object_color == "black")
      obstacle_detect(i, obj_color);
    if(object_color == "white")
      pipe_detect(i, obj_color);
    totem_detect(i, obj_color);
  }
  if(debug) if(obj_color == show_screen) cv::imshow("color", color);
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
  if (src.empty() || depth_mat.empty()) return;
  //Start the shape detection code
  cv::blur(src,src,Size(1,1));
  cv::cvtColor(src,hsv,COLOR_BGR2HSV); //HSV out
  cv::cvtColor(src,hls,COLOR_BGR2HLS); //HSL out
  width = src.cols;
  height = src.rows;
  //Find stuffs in each color
  do_inrange("blue", blue_low_lim, blue_up_lim);
  do_inrange("green", green_low_lim, green_up_lim);
  do_inrange("yellow", yellow_low_lim, yellow_up_lim);
  do_inrange("red", red_low_lim, red_up_lim, red_low_lim_wrap, red_up_lim_wrap);
  do_inrange("black", black_low_lim, black_up_lim);
  do_inrange("white", white_low_lim, white_up_lim);
  //Show output on screen in debug mode
  if(debug) 
  {
    cv::imshow("src", src);
  }
}

void depthCb(const sensor_msgs::ImageConstPtr& depth_msg)
{
	try
	{
		depth_ptr = cv_bridge::toCvCopy(depth_msg, "32FC1");
	  //refresh depth image
	  depth_mat = depth_ptr->image;
	}
	catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void dynamic_configCb(robotx_vision::detectionConfig &config, uint32_t level) 
{
  min_area = config.min_area;
  show_screen = config.show_screen;
  //Process appropriate parameter for object shape/color
  blue_low_lim = cv::Scalar(config.blue_H_low,config.blue_S_low,config.blue_V_low);
  blue_up_lim = cv::Scalar(config.blue_H_high,config.blue_S_high,config.blue_V_high);

  green_low_lim = cv::Scalar(config.green_H_low,config.green_S_low,config.green_V_low);
  green_up_lim = cv::Scalar(config.green_H_high,config.green_S_high,config.green_V_high);

  yellow_low_lim = cv::Scalar(config.yellow_H_low,config.yellow_S_low,config.yellow_V_low);
  yellow_up_lim = cv::Scalar(config.yellow_H_high,config.yellow_S_high,config.yellow_V_high);

  red_low_lim = cv::Scalar(config.red_H_low1, config.red_S_low, config.red_V_low);
  red_up_lim = cv::Scalar(config.red_H_high1, config.red_S_high, config.red_V_high);
  red_low_lim_wrap = cv::Scalar(config.red_H_low2, config.red_S_low, config.red_V_low);
  red_up_lim_wrap = cv::Scalar(config.red_H_high2, config.red_S_high, config.red_V_high);

  white_low_lim = cv::Scalar(config.white_H_low,config.white_L_low,config.white_S_low);
  white_up_lim = cv::Scalar(config.white_H_high,config.white_L_high,config.white_S_high);

  black_low_lim = cv::Scalar(config.black_H_low,config.black_S_low,config.black_V_low);
  black_up_lim = cv::Scalar(config.black_H_high,config.black_S_high,config.black_V_high);
  
  ROS_INFO("Reconfigure Requested.");
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "zed_detection");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("camera_link", camera_link);
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("subscribed_depth_topic", subscribed_depth_topic);
  pnh.getParam("published_topic", published_topic);
  pnh.getParam("debug", debug);
  //Processing object parameters for MarkerArray
  def_orientation.x = 0;
  def_orientation.y = 0;
  def_orientation.z = 0;
  def_orientation.w = 0;
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
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>(published_topic, 1000);
  ros::Rate r(30);
  //...and ROS transform listener
  tf_listener = new tf::TransformListener();
  while (nh.ok())
  {
  	//Do callbacks
  	ros::spinOnce();
    //Publish every object detected
    pub.publish(object);
    //Reinitialize the object counting vars
    object.markers.clear();
    //Loop
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}