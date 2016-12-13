#include <ros/ros.h>
#include "serial/serial.h"
#include "std_msgs/Int8.h"
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
  double frequency=0;
  int amplitude=0;


  double given_frequency;
  int threshold_amp;
  n.getParam("/hydrophone/threshold_amp", threshold_amp);
  n.getParam("/hydrophone/frequency", given_frequency);


  ros::Rate loop_rate(20);
  ser=new Serial("/dev/USBdue", 115200, serial::Timeout::simpleTimeout(250));
  ros::Publisher hydro_pub = n.advertise<std_msgs::Int8> ("hydrophone", 1000);

  while (ros::ok())
  {
    i=0;
    pos=0;
    msg=ser->readline();
    ROS_INFO("%s",msg.c_str());
    std_msgs::Int8 is_detected;


    while((pos=msg.find(delimiter))!=std::string::npos){
      token=msg.substr(0,pos);
      s[i]=::atof(token.c_str());
      msg.erase(0,pos+delimiter.length());
      i++;
    }
   
    s[1]=::atof(msg.c_str());

    if (s[1]!=0){
      amplitude=s[0];
      frequency=s[1];
      if((frequency<given_frequency+1000 && frequency > given_frequency-1000) && amplitude < threshold_amp){
        is_detected.data=1;
        hydro_pub.publish(is_detected);  
      }
    }
    else{
      is_detected.data=0;
    }   
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
