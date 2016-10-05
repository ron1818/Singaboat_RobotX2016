#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

//given cmd_val, publish motor_val. do base_controller with PID 
//reinaldo 2-10-16	

double ref_linear, ref_angular, act_linear, act_angular;

void cmdCallback(const geometry_msgs::Twist& msg)
{
  //assign reference velocity
  ref_linear=msg.linear.x;
  ref_angular=msg.angular.z;
}


void odomCallback(const nav_msgs::Odometry& msg)
{
  //update odometry values, requires linear velocity and angular velocity about USV's body frame
  act_linear=msg.twist.twist.linear.x;
  act_angular=msg.twist.twist.angular.z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller");
  ros::NodeHandle nh;

  //publish (forward, turn, 0) range -500 to 500, for arduino
  ros::Publisher motor_pub = nh.advertise<geometry_msgs::Vector3>("/motor_val", 100);
  //subscribe to cmd_vel 
  ros::Subscriber cmd_sub= nh.subscribe("/cmd_vel", 100, cmdCallback);
  //subscribe to odometry
  ros::Subscriber odom_sub = nh.subscribe("/odom", 100, odomCallback);

  bool reverse="False";

  double forward_P, forward_I, forward_D, angular_P, angular_I, angular_D;
  double f_error=0.0, f_derivator=0.0, f_integrator=0.0, f_I_max=500.0, f_I_min=-500.0;
  double a_error=0.0, a_derivator=0.0, a_integrator=0.0, a_I_max=500.0, a_I_min=-500.0;

  double f_set_point=0.0;
  double a_set_point=0.0;

  geometry_msgs::Vector3 motor_val;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    //check if reverse
    nh.getParam("/motor_param/reverse", reverse); 
    //get pid parameters
    nh.getParam("/motor_param/forward_P", forward_P); //values 0-2500 if linear 0.2
    nh.getParam("/motor_param/forward_I", forward_I);
    nh.getParam("/motor_param/forward_D", forward_D);
    nh.getParam("/motor_param/forward_derivator", f_derivator);
    nh.getParam("/motor_param/forward_integrator", f_integrator);
    
    
    nh.getParam("/motor_param/angular_P", angular_P); //values 0-500 if angular 1
    nh.getParam("/motor_param/angular_I", angular_I); 
    nh.getParam("/motor_param/angular_D", angular_D); 
    nh.getParam("/motor_param/angular_derivator", a_derivator);
    nh.getParam("/motor_param/angular_integrator", a_integrator);
    
    //reverse direction

    if (reverse)
    {
       ref_linear=-ref_linear;
    }

    //PID for linear
    f_error=ref_linear-act_linear;
    
    motor_val.x=forward_P*f_error+forward_D*(f_error-f_derivator);
    f_derivator=f_error;
    f_integrator=f_integrator+f_error;
 
    if (f_integrator>f_I_max)
    {
      f_integrator=f_I_max;
    }
    else if (f_integrator<f_I_min)
    {
      f_integrator=f_I_min;
    }
   
    motor_val.x=motor_val.x+forward_I*f_integrator;


    //PID for angular
    a_error=ref_angular-act_angular;
    
    motor_val.y=angular_P*a_error+angular_D*(a_error-a_derivator);
    a_derivator=a_error;
    a_integrator=a_integrator+a_error;
 
    if (a_integrator>a_I_max)
    {
      a_integrator=a_I_max;
    }
    else if (a_integrator<a_I_min)
    {
      a_integrator=a_I_min;
    }
   
    motor_val.y=motor_val.y+angular_I*a_integrator;
    motor_val.y=-motor_val.y;
    

    //apply saturation
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
    

    //publish to arduino
    motor_pub.publish(motor_val);
    ros::spinOnce();

    loop_rate.sleep();
   }

  return 0;

}


