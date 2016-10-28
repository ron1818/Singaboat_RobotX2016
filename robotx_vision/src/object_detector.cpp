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

//Variables to give info back to ROS from OpenCV
vector<robotx_vision::object_detection> object;

//OpenCV image processing vars (used only within opencv object detection)
cv::Mat src, hsv;
cv::Mat lower_hue_range;
cv::Mat upper_hue_range;
cv::Mat red,green;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
std::vector<std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::Rect r;
cv::RotatedRect mr;
int height, width;
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
  cv::blur(*bgr,*bgr,Size(3,3));
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

void totem_detect_red()
{
  //Filter red color
  cv::inRange(hsv, cv::Scalar(0, 170, 150), cv::Scalar(10, 255, 255), lower_hue_range);
  cv::inRange(hsv, cv::Scalar(160, 170, 150), cv::Scalar(179, 255, 255), upper_hue_range);
  cv::addWeighted(lower_hue_range,1.0,upper_hue_range,1.0,0.0,red);
  //Reduce noise
  reduce_noise(&red);
  //Finding shapes
  cv::findContours(red.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < 500) continue;
    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    r = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    double area = cv::contourArea(contours[i]);
    double r_area = (mr.size).height*(mr.size).width;
    double hull_area = contourArea(hull);

    if( ((fabs(r.height/r.width - 1.9) < eps ) && (fabs(area/r_area - 0.65) < eps) && (fabs(area/hull_area - 0.85) < eps)) || (fabs(r.height/r.width) > (1.9-eps)) && ((area/r_area - 0.8) < eps))
    { //If a red totem is detected
      cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 8, 8, 0);
      object.push_back(object_return(frame_id,"totem","red",r));         //Push the object to the vector
    }
  }
}

void totem_detect_green()
{
  //Filter green color
  cv::inRange(hsv, cv::Scalar(70, 180, 90), cv::Scalar(90, 255, 255), green);
  //Reduce noise
  reduce_noise(&green);
  //Finding shapes
  cv::findContours(green.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  //Detect shape for each contour
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if(std::fabs(cv::contourArea(contours[i])) < 2000) continue;
    //Calculate areas of object
    vector<Point> hull;
    convexHull(contours[i], hull, 0, 1);
    r = cv::boundingRect(contours[i]);
    mr = cv::minAreaRect(contours[i]);

    double area = cv::contourArea(contours[i]);
    double r_area = (mr.size).height*(mr.size).width;
    double hull_area = contourArea(hull);
    //Detect
    if((fabs(r.height/r.width) > (1.9-eps)) && (hull_area/r_area > (0.95-eps)))
    { //If a green totem is detected
      cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 8, 8, 0);
      object.push_back(object_return(frame_id,"totem","green",r));         //Push the object to the vector
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
  width = src.cols;
  height = src.rows;

  totem_detect_red();
  totem_detect_green();

  //Show output on screen
  //ROS_INFO("Node is working.");
  cv::imshow("src", src);
}

int main(int argc, char** argv)
{
  //Initiate node
  ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  pnh.getParam("frame_id", frame_id);
  pnh.getParam("output_topic_name", output_topic_name);

  //Initiate windows
  cv::namedWindow("src",WINDOW_NORMAL);
  cv::startWindowThread();

  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);

  //...and ROS publisher
  ros::Publisher pub = nh.advertise<robotx_vision::object_detection>(output_topic_name, 1000);
  ros::Rate r(30);

  while (nh.ok())
  {
    for(vector<robotx_vision::object_detection>::iterator it = object.begin(); it != object.end(); it++)
    {
      pub.publish(*it);
    }
    //Reinitialize the object counting vars
    object.clear();

    ros::spinOnce();
    r.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}