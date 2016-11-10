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
cv::Mat color;
cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
cv::Rect r;
cv::RotatedRect mr;
int height, width;
int min_area = 1500;
double area, m_area;
const double eps = 0.15;

//Code scanner vars
vector<std::string> color_sequence;
double sample_area;

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

int is_white(cv::Point2f vtx)
{
  const double ref = 10;
  int x = (ref+1)*vtx.x/ref - mr.center.x/ref;
  int y = (ref+1)*vtx.y/ref - mr.center.y/ref;
  Vec3b pixel = hls.at<Vec3b>(cv::Point(x,y));
  if((int)pixel.val[1]>200) return 1;
  return 0;
}

int check_white_bounded()
{
  cv::Point2f vtx[4];
  mr.points(vtx);
  if(is_white(vtx[1]) && is_white(vtx[2]) && is_white(vtx[3])) return 1;
  return 0;
}

int rect_detect(std::string obj_color, cv::Scalar up_lim, cv::Scalar low_lim, cv::Scalar up_lim_wrap = cv::Scalar(0,0,0), cv::Scalar low_lim_wrap = cv::Scalar(0,0,0))
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
    //Traverse through contours and fine the object
    for (int i = 0; i < contours.size(); i++)
    {
      r = cv::boundingRect(contours[i]);
      mr = cv::minAreaRect(contours[i]);
      if(check_white_bounded()) //Check if object is in front of a white background
      {
        area = cv::contourArea(contours[i]);
        m_area = (mr.size).height*(mr.size).width;
        
        if((fabs(1.0-area/m_area) < eps) && (r.height > r.width))
        {
          cv::rectangle(src, r.tl(), r.br()-cv::Point(1,1), cv::Scalar(0,255,255), 4, 8, 0);
          object.push_back(object_return(frame_id,"code",obj_color,r));         //Push the object to the vector
          return 1;
        }
      }
    }
    return 0;
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

  //Detect a black rect first
  if(rect_detect("black", cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 20)))
  {
    sample_area = m_area; //Register the area of the black rect for reference
    color_sequence.clear();
  }
  //Detecting other colors depending on the bgyr[] array assignment
  //BLUE
  if(rect_detect("blue", cv::Scalar(90, 250, 90), cv::Scalar(130, 255, 255)) && (fabs(m_area/sample_area - 1) < (eps+0.1)))
    if(color_sequence.empty() || (color_sequence.back() != "blue"))
      color_sequence.push_back("blue");
  //GREEN
  if(rect_detect("green", cv::Scalar(50, 200, 200), cv::Scalar(80, 255, 255)) && (fabs(m_area/sample_area - 1) < (eps+0.1)))
    if(color_sequence.empty() || (color_sequence.back() != "green"))
      color_sequence.push_back("green");
  //YELLOW
  if(rect_detect("yellow", cv::Scalar(20, 80, 100), cv::Scalar(30, 255, 255)) && (fabs(m_area/sample_area - 1) < (eps+0.1)))
    if(color_sequence.empty() || (color_sequence.back() != "yellow"))
      color_sequence.push_back("yellow");
  //RED
  if(rect_detect("red", cv::Scalar(0, 80, 100), cv::Scalar(10, 255, 255), cv::Scalar(165, 80, 100), cv::Scalar(176, 255, 255)) && (fabs(m_area/sample_area - 1) < (eps+0.1)))
    if(color_sequence.empty() || (color_sequence.back() != "red"))
      color_sequence.push_back("red");

  if(color_sequence.size()==3)
  {
    ros::param::set("/gui/color1", color_sequence[0]);
    ros::param::set("/gui/color2", color_sequence[1]);
    ros::param::set("/gui/color3", color_sequence[2]);
    ros::shutdown(); //Shutdown when job is done
  }
  //Show output on screen
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
  cv::resizeWindow("src",640,480);
  cv::startWindowThread();

  //Start ROS subscriber...
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(subscribed_image_topic, 1, imageCb);

  //...and ROS publisher
  ros::Publisher pub = nh.advertise<robotx_vision::object_detection>(output_topic_name, 1000);
  ros::Rate rate(5);

  while (nh.ok())
  {
    //Publish every object detected
    for(vector<robotx_vision::object_detection>::iterator it = object.begin(); it != object.end(); it++)
      pub.publish(*it);
    //Reinitialize the object counting vars
    object.clear();
    //Do imageCb function and go to sleep
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}