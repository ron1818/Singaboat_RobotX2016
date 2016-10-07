#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

//given cmd_val, publish motor_val. do base_controller with PID 	

double linear, angular;

void cmdCallback(const geometry_msgs::Twist& msg)
{
  linear=msg.linear.x;
  angular=msg.angular.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_motor");
  ros::NodeHandle nh;

  //publish (forward, turn, 0) range -500 to 500, for arduino
  ros::Publisher motor_pub = nh.advertise<geometry_msgs::Twist>("/motor_val", 1000);
  //subscribe to cmd_vel command
  ros::Subscriber cmd_sub= nh.subscribe("/cmd_vel_mux/input/teleop", 1000, cmdCallback);

  int forward_ratio, angular_ratio;
  int right_calib, left_calib;
  bool isReverse="false";
  geometry_msgs::Twist motor_val;

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    nh.getParam("/motor_param/forward", forward_ratio); //values 0-2500 if linear 0.2
    nh.getParam("/motor_param/angular", angular_ratio); //values 0-500 if angular 1
    nh.getParam("/motor_param/right_calibration", right_calib); //value set to 0, offset 
    nh.getParam("/motor_param/left_calibration", left_calib); //value set to 0
    nh.getParam("/motor_param/reverse", isReverse);

    motor_val.linear.y=right_calib; //calibration factor for right motor
    motor_val.linear.z=left_calib; //calibration factor for left motor

    motor_val.linear.x=forward_ratio*linear;
    motor_val.angular.z=-1*angular_ratio*angular;

    if (isReverse)
    {
      motor_val.linear.x=-motor_val.linear.x;
    }

    if (motor_val.linear.x>500)
    {
      motor_val.linear.x=500;
    }
    else if (motor_val.linear.x<-500)
    {
      motor_val.linear.x=-500;
    }

    if (motor_val.angular.z>500)
    {
      motor_val.angular.z=500;
    }
    else if (motor_val.angular.z<-500)
    {
      motor_val.angular.z=-500;
    }
    
    motor_pub.publish(motor_val);
    ros::spinOnce();

    loop_rate.sleep();
   }

  return 0;

}


