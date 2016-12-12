#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "serial/serial.h"
#include "geometry_msgs/Vector3.h"
//#include "sensor_msgs/Imu.msg"
#include <tf/transform_broadcaster.h>
#include <cmath>


using std::string;
using serial::Serial;

int main(int argc, char **argv)
{
  Serial *ser;
  ros::init(argc, argv, "hydrophone");
  ros::NodeHandle n;
  std::string msg;
  std::string token;
  std::string delimiter=",";
  size_t pos=0;
  double s[2]={};
  int i=0;


  ros::Rate loop_rate(20);
  ser=new Serial("/dev/USBdue", 115200, serial::Timeout::simpleTimeout(250));


  ros::Publisher hydro_pub = n.advertise<geometry_msgs::Vector3> ("hydrophone/data_raw", 1000);

  while (ros::ok())
  {
    i=0;
    pos=0;
    msg=ser->readline();
    ROS_INFO("%s",msg.c_str());
    geometry_msgs::Vector3 raw_msg;


    while((pos=msg.find(delimiter))!=std::string::npos){
      token=msg.substr(0,pos);
      s[i]=::atof(token.c_str());
      msg.erase(0,pos+delimiter.length());
      i++;
    }
   
   
    s[1]=::atof(msg.c_str());

    if (s[1]==0){
      s[0]=0;
    }   


    raw_msg.x=s[0];
    raw_msg.y=s[1];
   
    hydro_pub.publish(raw_msg);

    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
