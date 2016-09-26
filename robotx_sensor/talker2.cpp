#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>


using std::string;
using serial::Serial;

int main(int argc, char **argv)
{
    Serial *ser;
    ros::init(argc, argv, "talker2");
    ros::NodeHandle n;
    std::string msg;
    std::string token;
    std::string delimiter=",";
    size_t pos=0;
    double s[11]={};
    int i=0;
    double rpy[3]=0;
    double accel[3]=0;
    double gyro[3]=0;
    geometry_msgs::Quaternion quat;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu> ("imu/data", 1000);
    ros::Rate loop_rate(50);
    ser=new Serial("/dev/ttyUSB0", 57600, serial::Timeout::simpleTimeout(250));

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
        s[6]=::atof(msg.c_str());

        rpy=s[0:2];
        gyro=s[4:6];
        accel=s[8:10];


        quat= tf::createQuaternionFromRPY(rpy[0], rpy[1], rpy[2]);


        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = frame_id;
        imu_msg.orientation =quat;
        imu_msg.angular_velocity.x = gyro[0];
        imu_msg.angular_velocity.y = gyro[1];
        imu_msg.angular_velocity.z = gyro[2];
        imu_msg.linear_acceleration.x =accel[0];
        imu_msg.linear_acceleration.y =accel[1];
        imu_msg.linear_acceleration.z =accel[2];


        imu_pub.publish(imu_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
