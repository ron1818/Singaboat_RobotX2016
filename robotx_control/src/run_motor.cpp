#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

double linear, angular;

void cmdCallback(const geometry_msgs::Twist& msg)
{
  linear=msg.linear.x;
  angular=msg.angular.z;
}

void speedCallback(const geometry_msgs::Vector3& msg)
{
  ROS_INFO("right: %f , left: %f", msg.x, msg.y);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_motor");
  ros::NodeHandle nh;

  //publish (forward, turn, 0) range -500 to 500, for arduino
  ros::Publisher motor_pub = nh.advertise<geometry_msgs::Vector3>("/motor_val", 1000);
  //subscribe to cmd_vel command
  ros::Subscriber cmd_sub= nh.subscribe("/cmd_vel", 1000, cmdCallback);
  //subscribe to ??
  ros::Subscriber speed_sub = nh.subscribe("/chatter", 1000, speedCallback);

  int forward_ratio, angular_ratio;
  geometry_msgs::Vector3 motor_val;

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    nh.getParam("/motor_param/forward", forward_ratio); //values 0-2500 if linear 0.2
    nh.getParam("/motor_param/angular", angular_ratio); //values 0-500 if angular 1

    motor_val.x=forward_ratio*linear;
    motor_val.y=-1*angular_ratio*angular;

    if (motor_val.x>500)
    {
      motor_val.x=500;
    }
    else if (motor_val.x<-500)
    {
      motor_val.x=-500;
    }

    if (motor_val.y>500)
    {
      motor_val.y=500;
    }
    else if (motor_val.y<-500)
    {
      motor_val.y=-500;
    }

    motor_pub.publish(motor_val);
    ros::spinOnce();

    loop_rate.sleep();
   }

  return 0;

}


