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

//Image transport vars
cv_bridge::CvImagePtr cv_ptr;

//Image processing vars
bool newImage = false;

std::string nodeName = "my_record";
std::string output = "/home/echo/record/output_bowright.avi";
Mat src;
int frame_width = 1280;
int frame_height = 720;
cv::VideoWriter video(output,CV_FOURCC('M','J','P','G'),30, Size(frame_width,frame_height),true);

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
  if (src.empty())
  {
    ROS_INFO("Cannot load image, looping until image is loaded...");
    return;
  }

  video.write(src);
  cv::imshow("src", src);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;
  //output = argv[1];

  //Initiate windows
  cv::namedWindow("src",CV_WINDOW_NORMAL);
  cv::startWindowThread();

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
