/*This node funtion(s):
	+ Detect symbols
  + Detect markers
  + Detect obstacles
*/

//ROS libs
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <robotx_vision/shapeConfig.h>
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
//Dynamic reconfigure vars
int min_area = 300;
int low_threshold, high_threshold;
//OpenCV image processing method dependent vars 
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Mat src, gray, edge;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(4,4));
cv::Rect rect;
cv::RotatedRect mr;
int height, width;
double area, r_area, mr_area, hull_area;
const double eps = 0.15;

//Functions
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

void totem_detect(int i)
{
  object_shape = "totem";
  //if(object_color == "red")
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

void pipe_detect(int i)
{
  object_shape = "pipe";
  if(((double)rect.width/rect.height > 2) && (fabs(area/mr_area - 1) < eps))
    object_found();
}

void tri_detect(int i)
{
  object_shape = "triangle";
  if((std::fabs(area/mr_area - 3.141593/4) < 0.1) && (std::fabs(area/hull_area - 1) < 0.05))
    object_found();
}

void cir_detect(int i)
{
  object_shape = "circle";
  if((fabs(area/mr_area - 0.5) < 0.07 && (std::fabs(area/hull_area - 1) < 0.05)) /*&& cv::isContourConvex(approx)*/)
    object_found();
}

void cru_detect(int i)
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
  if (src.empty()) return;
  //Start the shape detection code
  cv::blur(src,src,Size(1,1));
  cv::cvtColor(src,gray,COLOR_BGR2GRAY);
  width = src.cols;
  height = src.rows;
  //Detect stuffs
  cv::Canny(gray, edge, low_threshold, high_threshold, 5);
  //Finding shapes
  cv::findContours(edge.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
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

    if(object_shape == "triangle") tri_detect(i);
    if(object_shape == "circle") cir_detect(i);
    if(object_shape == "cruciform") cru_detect(i);
    if(object_shape == "pipe") pipe_detect(i);
    if(object_shape == "totem") totem_detect(i);
  }
  //Show output on screen in debug mode
  if(debug) 
  {
    cv::imshow("src", src);
    cv::imshow("edge", edge);
  }
}

void dynamic_configCb(robotx_vision::shapeConfig &config, uint32_t level) 
{
  min_area = config.min_area;
  low_threshold = config.low_threshold;
  high_threshold = config.high_threshold;
  
  ROS_INFO("Reconfigure Requested.");
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "detection_roi");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("object_shape", object_shape);
  pnh.getParam("object_color", object_color);
  pnh.getParam("debug", debug);
  pnh.getParam("published_topic", published_topic);
  //Initiate windows
  if(debug)
  {
   /* cv::namedWindow("edge",WINDOW_AUTOSIZE);
    cv::namedWindow("src",WINDOW_AUTOSIZE);*/
    cv::namedWindow("src",WINDOW_NORMAL);
    cv::resizeWindow("src",640,480);
    cv::namedWindow("edge",WINDOW_NORMAL);
    cv::resizeWindow("edge",640,480);
    cv::startWindowThread();
  }
  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);
  //Dynamic reconfigure
  dynamic_reconfigure::Server<robotx_vision::shapeConfig> server;
  dynamic_reconfigure::Server<robotx_vision::shapeConfig>::CallbackType f;
  f = boost::bind(&dynamic_configCb, _1, _2);
  server.setCallback(f);
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