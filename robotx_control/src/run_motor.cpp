#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include <dynamic_reconfigure/server.h>
#include <robotx_control/StartMotorConfig.h>

//given cmd_vel, publish motor_val. do base_controller with PID 	

double linear, angular;
double forward_ratio=375, angular_ratio=200;

void cmdCallback(const geometry_msgs::Twist& msg)
{
  linear=msg.linear.x;
  angular=msg.angular.z;
}

void dynamic_callback(robotx_control::StartMotorConfig &config, uint32_t level){
  ROS_INFO("Reconfigure Request: %f %f", config.motor_param_forward, config.motor_param_angular);

  forward_ratio=config.motor_param_forward;
  angular_ratio=config.motor_param_angular;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "start_motor");
  ros::NodeHandle nh;

  //publish (forward, turn, 0) range -500 to 500, for arduino
  ros::Publisher motor_pub = nh.advertise<geometry_msgs::Twist>("/motor_val", 1000);
  //subscribe to cmd_vel command
  ros::Subscriber cmd_sub= nh.subscribe("/cmd_vel", 1000, cmdCallback);


  bool isReverse=false;
  geometry_msgs::Twist motor_val;
    //dynamic variable
  dynamic_reconfigure::Server<robotx_control::StartMotorConfig> server;
  dynamic_reconfigure::Server<robotx_control::StartMotorConfig>::CallbackType f;

  f=boost::bind(&dynamic_callback, _1, _2);
  server.setCallback(f);

  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    nh.getParam("serial_arduino_node/motor_param/reverse", isReverse);

    motor_val.linear.x=linear*forward_ratio;
    motor_val.angular.z=-1*angular*angular_ratio; 


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


