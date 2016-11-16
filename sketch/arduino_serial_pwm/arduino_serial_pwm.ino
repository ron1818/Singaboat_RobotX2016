
/* Given forward and turn commands(through ROS or RC), run motors accordingly
 *
 * 
 * in ROS:
 * 1. run roscore
 * 2. in a new tab(ctrl+shift+t) 
 *    rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600 
 * 3. in another tab
 *    rostopic echo chatter
 * 4. in another tab
 *    rostopic pub motor_motion geometry_msgs/Vector3 '-500.0' '500.0'0.0'    
 *    these are forwardROS and turnROS variables (range -500 to 500)
 * 5. publishes hydrophone analog value
 * 
 * 
 *
 */
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <Servo.h>

#define throttle_pin 5
#define steering_pin 6

ros::NodeHandle nh;


Servo throttle;
Servo steering;

int forwardROS=1500;
int turnROS=1500;

const int max_pwm = 1900;
const int min_pwm = 1100;
const int neutral_pwm = 1500;

int throttle_pwm = neutral_pwm;
int steering_pwm = neutral_pwm;

geometry_msgs::Vector3 movement; //for chatter

void changeVar(const geometry_msgs::Twist& input){
  forwardROS=map(input.linear.x, -500, 500, 1100, 1900);
  turnROS=map(input.angular.z, -500, 500, 1100, 1900);
}

ros::Subscriber<geometry_msgs::Twist> sub("/motor_val", &changeVar);
ros::Publisher chatter("chatter", &movement);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

  pinMode(throttle_pin, OUTPUT);
  pinMode(steering_pin, OUTPUT);

  throttle.attach(throttle_pin);
  steering.attach(steering_pin);

  throttle.writeMicroseconds(neutral_pwm);
  steering.writeMicroseconds(neutral_pwm);
  
  Serial.begin(57600); // Pour a bowl of Serial
}

void loop() {

  throttle.writeMicroseconds(forwardROS);
  steering.writeMicroseconds(turnROS);
  movement.x=forwardROS; //0=>fastest forward, 255 =>fastest backwards, 120 =>stops
  movement.y=turnROS;
  chatter.publish(&movement);
  nh.spinOnce();
  delay(100);
}

