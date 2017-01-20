/*  motor rudder controller to differential drive
    ren ye
    2016-08-11
    init
    2016-11-10
    use RC controller and pwm generator to control
    use Serial library
    2016-11-29
    use ros again to send pwm signals
    avoid pwm chip, save space

    TACTIC 2.4GHz with TR324 receiver
    TR324     Arduino
    Ch1   ->  D7
    Ch2   ->  D8
    Ch3   ->  D2 (ack) interrupt input
    VCC   <-  5V
    GND   <-  GND

    ##### outdated###
    PCA9685 PWM generator
    9685      74157
    Ch0   ->  D5
    Ch1   ->  D6
    #################
    

    Arduino   74157
    D4     ->  ~sel onoff output
    D5     ->  A_1
    D6     ->  A_2
    D9     ->  B_1
    D10    ->  B_2
    V5     ->  VCC
    GND    ->  GND
*/

# include <Servo.h>
#include <ros.h>
// #include <ros/time.h>
#include <Arduino.h>
// #include <std_msgs/String.h>
#include <std_msgs/Float32.h>
// use uint8multiarray
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

Servo left_rc;
Servo right_rc;
Servo left_ros;
Servo right_ros;

// set up ros nodes and publishers
std_msgs::Float32MultiArray control_msg;
// std_msgs::Float32 pwm_ack_msg;


ros::Publisher pub("control_fb", &control_msg);

ros::NodeHandle nh;

#define SteeringPin 7  // rudder
#define ThrottlePin 8  // propeller
#define AckInPin 2   // ack input pin
#define AckOutPin 4   // ack output pin
#define left_thruster_ros 5  // left thruster from ros
#define right_thruster_ros 6 // right thruster from ros
#define left_thruster 9 // left thruster
#define right_thruster 10   // right thruster
#define LED 13  // led pin

float timer_period;
unsigned int T, T_reset, T_prev, T_offset, T_min, T_max;

const int max_pwm = 1900;
const int min_pwm = 1100;
const int ack_threshold1 = 900;
const int ack_threshold2 = 1300;
const int neutral_pwm = 1500;
const float wheel_sep = 0.6;
const float DT = 0.02;

int linear_x_center = neutral_pwm;
int angular_z = neutral_pwm;

int left_pwm = neutral_pwm;
int right_pwm = neutral_pwm;
int left_pwm_ros = neutral_pwm;
int right_pwm_ros = neutral_pwm;


// pwm callback
void pwmCb(const std_msgs::UInt16MultiArray& msg) {
  // read pwm and convert to pwm
  left_pwm_ros = msg.data[0];
  right_pwm_ros = msg.data[1];
}

ros::Subscriber<std_msgs::UInt16MultiArray> pwm_sub("/pwm", &pwmCb );


void setup() {
  // Serial.begin(9600);
  pinMode(SteeringPin, INPUT);
  pinMode(ThrottlePin, INPUT);
  pinMode(AckInPin, INPUT);
  pinMode(left_thruster_ros, OUTPUT);
  pinMode(right_thruster_ros, OUTPUT);
  pinMode(left_thruster, OUTPUT);
  pinMode(right_thruster, OUTPUT);
  pinMode(AckOutPin, OUTPUT);
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  digitalWrite(AckOutPin, HIGH);

  // attach esc
  left_rc.attach(left_thruster);
  right_rc.attach(right_thruster);
  left_ros.attach(left_thruster_ros);
  right_ros.attach(right_thruster_ros);


  left_rc.writeMicroseconds(neutral_pwm);
  right_rc.writeMicroseconds(neutral_pwm);
  left_ros.writeMicroseconds(neutral_pwm);
  right_ros.writeMicroseconds(neutral_pwm);

  nh.initNode();
  nh.subscribe(pwm_sub);

  control_msg.layout.dim = (std_msgs::MultiArrayDimension *)
                        malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  control_msg.layout.dim->label = "arduino_reading";
  control_msg.layout.dim->size = 5;
  control_msg.layout.data_offset = 0;
  control_msg.data = (float *)malloc(sizeof(float) * 4);
  control_msg.data_length = 5;

  nh.advertise(pub);
  delay(2000);
}

long action_timer;
long publisher_timer;

void loop() {
  int sel = 1;
  // read AckInPin check if it has pwm input
  int ack_pwm = pulseIn(AckInPin, HIGH, 25000);


  Serial.print(ack_pwm);
  Serial.print(", ");
  if (ack_pwm >= ack_threshold1 && ack_pwm <= ack_threshold2) { // ack engaged when ~970
    digitalWrite(AckOutPin, HIGH);
    sel = 1;
    int tmp_angular_z = pulseIn(SteeringPin, HIGH);
    int tmp_linear_x_center = pulseIn(ThrottlePin, HIGH);
    
    if (tmp_linear_x_center > 500) {
      linear_x_center = tmp_linear_x_center;
    }
    if (tmp_angular_z > 500) {
      angular_z = tmp_angular_z;
    }
    /* Serial.print(linear_x_center);
    Serial.print(", ");
    Serial.println(angular_z);*/

    conv_differential(linear_x_center, angular_z);
    // write to pwm motor pin
    /* Serial.print(left_pwm);
    Serial.print(", ");
    Serial.println(right_pwm); */
    left_rc.writeMicroseconds(left_pwm);
    right_rc.writeMicroseconds(right_pwm);
  } else {
    digitalWrite(AckOutPin, LOW);
    sel = 0;

    left_ros.writeMicroseconds(left_pwm_ros);
    right_ros.writeMicroseconds(right_pwm_ros);

    // Serial.println("from PCA9685");
    // 74ls157 will take pwm from pca9685
  }
  delay(50);

  // publish topic
  if (millis() > publisher_timer) {

    // publish battery and temperature
    control_msg.data[0] = left_pwm;
    control_msg.data[1] = right_pwm;
    control_msg.data[2] = left_pwm_ros;
    control_msg.data[3] = right_pwm_ros;
    control_msg.data[4] = sel;
    pub.publish(&control_msg);

    publisher_timer = millis() + DT;

  }
  nh.spinOnce();
  
}

void conv_differential(int linear_x_center, int angular_z) {
  int insensitive_value = 50;
  if (linear_x_center > (neutral_pwm + insensitive_value) ||
      angular_z > (neutral_pwm + insensitive_value) ||
      linear_x_center < (neutral_pwm - insensitive_value) ||
      angular_z < (neutral_pwm - insensitive_value)) {
    digitalWrite(LED, HIGH);
  } else {
    digitalWrite(LED, LOW);
  }
  int angular_offset = insensitive(angular_z) - neutral_pwm;
  left_pwm = linear_x_center - angular_offset;
  right_pwm = linear_x_center + angular_offset;
  left_pwm = insensitive(left_pwm);
  right_pwm = insensitive(right_pwm);
  
}

int insensitive(int pwm) {
  int insensitive_value = 50;
  pwm = constrain(pwm, min_pwm, max_pwm);

  if (pwm < (neutral_pwm + insensitive_value) &&
      pwm > (neutral_pwm - insensitive_value)) {
    return neutral_pwm;
  } else {
    return pwm;
  }
}


