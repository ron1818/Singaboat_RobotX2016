#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include "robotx_vision/object_detection.h"
#include <iostream>
#include <stdio.h>
#include <string>

using namespace std;

std::string subscribed_image_topic;

void Cb(robotx_vision::object_detection ob)
{
  ROS_INFO("Object detected at camera = %s",subscribed_image_topic.c_str());
  cout << "\tType:\t" << ob.type << endl;
  cout << "\tColor:\t" << ob.color << endl;
  cout << "\tTop angle:\t" << ob.angle_t << endl;
  cout << "\tBot angle:\t" << ob.angle_b << endl;
  cout << "\tLeft angle:\t" << ob.angle_l << endl;
  cout << "\tRight angle:\t" << ob.angle_r << endl;
  cout << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_subscriber");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  pnh.getParam("subscribed_image_topic", subscribed_image_topic);
  ros::Subscriber sub = n.subscribe(subscribed_image_topic, 1000, Cb);
  ros::spin();

  return 0;
}