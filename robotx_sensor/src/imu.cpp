#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "serial/serial.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
//#include "sensor_msgs/Imu.msg"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define Gravity 25.28
#define Gyro_gain 0.0012043


using std::string;
using serial::Serial;

int main(int argc, char **argv)
{
  Serial *ser;
  ros::init(argc, argv, "imu");
  ros::NodeHandle n;
  std::string msg;
  std::string token;
  std::string delimiter=",";
  size_t pos=0;
  double s[14]={};
  int i=0;
  boost::array<double, 9> covariance_ori = {0};
            covariance_ori[0] = 0.0025;
            covariance_ori[4] = 0.0025;
            covariance_ori[8] = 0.0025;
  boost::array<double, 9> covariance_an = {0};
            covariance_an[0] = 0.02;
            covariance_an[4] = 0.02;
            covariance_an[8] = 0.02;
  boost::array<double, 9> covariance_acc = {0};
            covariance_acc[0] = 0.04;
            covariance_acc[4] = 0.04;
            covariance_acc[8] = 0.04;
  geometry_msgs::Vector3 rpy;
  geometry_msgs::Vector3 gyro;
  geometry_msgs::Vector3 acc;

  sensor_msgs::MagneticField mag;
  geometry_msgs::Quaternion quat;

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu> ("middle_middle_imu/imu/data_raw", 1000);
  ros::Publisher mag_pub = n.advertise<sensor_msgs::MagneticField> ("middle_middle_imu/imu/mag", 1000);
  ros::Rate loop_rate(50);
  ser=new Serial("/dev/USBimu", 57600, serial::Timeout::simpleTimeout(250));

  std::string frame_id="imu_link";


  while (ros::ok())
  {
    i=0;
    pos=0;
    msg=ser->readline();
    ROS_INFO("%s",msg.c_str());
    msg.erase(0,5);
    sensor_msgs::Imu imu_msg;


    while((pos=msg.find(delimiter))!=std::string::npos){

      	token=msg.substr(0,pos);
   	s[i]=::atof(token.c_str());
   	msg.erase(0,pos+delimiter.length());
   	i++;
   }
   s[14]=::atof(msg.c_str());

   rpy.x=s[0]*M_PI/180;
   rpy.y=s[1]*M_PI/180;
   rpy.z=s[2]*M_PI/180;
   //ROS_INFO("%lf, %lf, %lf", rpy.x, rpy.y, rpy.z);
   gyro.x=s[4]*Gyro_gain;
   gyro.y=s[5]*Gyro_gain;
   gyro.z=s[6]*Gyro_gain;
   acc.x=s[8]/Gravity;
   acc.y=s[9]/Gravity;
   acc.z=s[10]/Gravity;
   mag.magnetic_field.x=s[12];
   mag.magnetic_field.y=s[13];
   mag.magnetic_field.z=s[14];



   //geometry_msgs::Quaternion quat= tf::createQuaternionFromRPY(double rpy.x, double rpy.y, double rpy.z);

    quat= tf::createQuaternionMsgFromRollPitchYaw(rpy.x, rpy.y, rpy.z);


    //static geometry_msgs::Quaternion quat=tf::createQuaternionFromRPY(rpy.x, rpy.y,rpy.z);


            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;
            imu_msg.orientation.x =quat.x;
            imu_msg.orientation.y =quat.y;
            imu_msg.orientation.z =quat.z;
            imu_msg.orientation.w =quat.w;
            imu_msg.angular_velocity.x = gyro.x;
            imu_msg.angular_velocity.y = gyro.y;
            imu_msg.angular_velocity.z = gyro.z;
            imu_msg.linear_acceleration.x =acc.x;
            imu_msg.linear_acceleration.y =acc.y;
            imu_msg.linear_acceleration.z =acc.z;
            imu_msg.orientation_covariance = covariance_ori;
            imu_msg.angular_velocity_covariance = covariance_an;
            imu_msg.linear_acceleration_covariance = covariance_acc;



   imu_pub.publish(imu_msg);
   mag_pub.publish(mag);

   ros::spinOnce();
   loop_rate.sleep();
  }

  return 0;
}
