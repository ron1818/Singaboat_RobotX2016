#!/usr/bin/env python
# reinaldo 2-12-16
# calibration for boat
# gives correction factor actual_vel/cmd_vel
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class MotionCalibrate(object):
    odom_x, odom_y, odom_yaw = 0, 0, 0

    def __init__(self):
        rospy.init_node('motion_calibrate', anonymous=True)
      
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("odom", Odometry, callback=self.odom_callback, queue_size=10)
        
        angular_ratio=list()

        ratio_1=self.rotate(0.2)
        rospy.sleep(1)
        ratio_2=self.rotate(-0.2)
        rospy.sleep(1)
        ratio_3=self.rotate(0.4)
        rospy.sleep(1)
        ratio_4=self.rotate(-0.4)
        angular_ratio.append(ratio_1)
        angular_ratio.append(ratio_2)
        angular_ratio.append(ratio_3)
        angular_ratio.append(ratio_4)


        angular_mean_ratio=sum(angular_ratio)/len(angular_ratio)
        print("angular mean ratio: ")
        print(angular_mean_ratio)

        linear_ratio=list()

        ratio_a=self.linear(0.5)
        rospy.sleep(2)
        ratio_b=self.linear(1)
        
        linear_ratio.append(ratio_a)
        linear_ratio.append(ratio_b)

        linear_mean_ratio=sum(linear_ratio)/len(linear_ratio)
        print("linear mean ratio: ")
        print(linear_mean_ratio)

        rospy.spin()


    def rotate(self, speed):

        msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, speed))
        angle_threshold=math.radians(10)
        start_time = rospy.get_time()
        initial_yaw = self.odom_yaw
        
        #turn one full round
        while not rospy.is_shutdown() and (rospy.get_time()-start_time<1 or math.fabs(math.atan2(math.sin(self.odom_yaw-initial_yaw), math.cos(self.odom_yaw-initial_yaw))) > angle_threshold):
            self.cmd_pub.publish(msg)
            rospy.sleep(0.2)

        final_time=rospy.get_time()        
        #publish empty twist to stop    
        self.cmd_pub.publish(Twist())

        duration=final_time-start_time

        actual_vel=2*math.pi/duration

        correction_ratio=actual_vel/speed
            
        return correction_ratio 

    def linear(self, speed):

        msg = Twist(Vector3(speed, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        #move forward 5 metres
        linear_threshold=5
        start_time = rospy.get_time()
        initial_pos = math.sqrt(self.odom_x**2+self.odom_y**2)
        
        #turn one full round
        while not rospy.is_shutdown() and math.fabs(math.sqrt(self.odom_x**2+self.odom_y**2)-initial_pos) < linear_threshold:
            self.cmd_pub.publish(msg)
            rospy.sleep(0.1)

        final_time=rospy.get_time()        
        #publish empty twist to stop    
        self.cmd_pub.publish(Twist())


        duration=final_time-start_time

        actual_vel=linear_threshold/duration

        correction_ratio=actual_vel/speed
            
        return correction_ratio      



    def odom_callback(self, msg):
        """ callback the subscribe, get pose data """
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, self.odom_yaw = euler_from_quaternion((x, y, z, w))

if __name__ == '__main__':
    try:
        motion_calibrate = MotionCalibrate()

    except KeyboardInterrupt, rospy.ROSInterruptException:
        pass
