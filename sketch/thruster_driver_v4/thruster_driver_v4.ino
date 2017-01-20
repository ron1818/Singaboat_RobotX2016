/*  motor rudder controller to differential drive
    ren ye
    2016-08-11
    init
    2016-11-10
    use RC controller and pwm generator to control
    use Serial library
    2017-01-13
    use flysky to control
    use mega
    send e-stop led status

    flysky ia6
    ia6     Arduino
    Ch1   ->  D7
    Ch2   ->  D8
    Ch5   ->  D9 (ack) pwm input


    Arduino   74157
    D4    ->  ~sel onoff output
    D5    ->  B_1
    D6    ->  B_2
    V5    ->  VCC
    GND   ->  GND
*/
#include <ros.h>
#include <Arduino.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Servo.h>

ros::NodeHandle nh;

Servo left_rc;
Servo right_rc;
Servo left_ros;
Servo right_ros;

#define SteeringPin 36  // rudder
#define ThrottlePin 38  // propeller
#define CH5 42   // mode select pin
#define SEL 44   // mux select pin
#define left_thrusterRC 5 // left thruster rc
#define right_thrusterRC 7   // right thruster rc
#define left_thrusterROS 6 // left thruster rc
#define right_thrusterROS 8   // right thruster rc
#define LED_ctrl1 46
#define LED_ctrl0 48

float timer_period;
unsigned int T, T_reset, T_prev, T_offset, T_min, T_max;

const int max_pwm = 1900;
const int min_pwm = 1100;
const int ack_threshold[4] = {900, 1200, 1600, 2000};
const int gap = 10;
const int neutral_pwm = 1500;
const float wheel_sep = 0.6;
const float DT = 0.02;

int linear_x_center = neutral_pwm;
int angular_z = neutral_pwm;

int left_pwmRC = neutral_pwm;
int right_pwmRC = neutral_pwm;
int left_pwmROS = neutral_pwm;
int right_pwmROS = neutral_pwm;

// pwm callback
void pwmCb(const std_msgs::UInt16MultiArray& msg) {
  // read pwm and convert to pwm
  left_pwmROS = msg.data[0];
  right_pwmROS = msg.data[1];
}

ros::Subscriber<std_msgs::UInt16MultiArray> pwm_sub("/pwm", &pwmCb );

void setup() {
  pinMode(SteeringPin, INPUT);
  pinMode(ThrottlePin, INPUT);
  pinMode(CH5, INPUT);
  pinMode(left_thrusterROS, OUTPUT);
  pinMode(right_thrusterROS, OUTPUT);
  pinMode(left_thrusterRC, OUTPUT);
  pinMode(right_thrusterRC, OUTPUT);
  pinMode(SEL, OUTPUT);
  pinMode(LED_ctrl1, OUTPUT);
  pinMode(LED_ctrl0, OUTPUT);

  digitalWrite(LED_ctrl1, LOW);
  digitalWrite(LED_ctrl0, LOW);
  digitalWrite(SEL, LOW);  // mux RC

  // Serial.begin(57600);
  
  // attach esc
  left_rc.attach(left_thrusterRC);
  right_rc.attach(right_thrusterRC);
  left_ros.attach(left_thrusterROS);
  right_ros.attach(right_thrusterROS);


  left_rc.writeMicroseconds(neutral_pwm);
  right_rc.writeMicroseconds(neutral_pwm);
  left_ros.writeMicroseconds(neutral_pwm);
  right_ros.writeMicroseconds(neutral_pwm);
  delay(1000);

  nh.initNode();
  nh.subscribe(pwm_sub);
  delay(1000);

}

long action_timer;
long publisher_timer;

void loop() {
  int sel = 1;
  // read AckInPin check if it has pwm input
  int mode_pwm = pulseIn(CH5, HIGH);


  /* Serial.print(mode_pwm);
  Serial.print(", "); */
  if (mode_pwm < (ack_threshold[0] - gap)) { // RC not connected
    sel = -1;
    digitalWrite(LED_ctrl1, LOW); //
    digitalWrite(LED_ctrl0, LOW); //
    digitalWrite(SEL, LOW);  // mux auto
    left_ros.writeMicroseconds(left_pwmROS);
    right_ros.writeMicroseconds(right_pwmROS);
  } else if (mode_pwm >= (ack_threshold[0] + gap) && mode_pwm <= (ack_threshold[1] - gap)) { // e-stop when <1300
    sel = 0;
    digitalWrite(LED_ctrl1, HIGH); // e-stop relay on
    digitalWrite(LED_ctrl0, HIGH); //
    digitalWrite(SEL, HIGH);  // mux ROS
    left_rc.writeMicroseconds(neutral_pwm);  // put neutral in case
    right_rc.writeMicroseconds(neutral_pwm);
  } else if (mode_pwm >= (ack_threshold[1] + gap) && mode_pwm <= (ack_threshold[2] - gap)) { // manual when ~1500
    sel = 1;
    digitalWrite(LED_ctrl1, HIGH); // e-stop relay on
    digitalWrite(LED_ctrl0, LOW); //
    digitalWrite(SEL, LOW);  // mux RC
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
    left_rc.writeMicroseconds(left_pwmRC);
    right_rc.writeMicroseconds(right_pwmRC);
  } else {  // auto
    digitalWrite(SEL, HIGH);  // mux ROS
    digitalWrite(LED_ctrl1, LOW); // e-stop relay on
    digitalWrite(LED_ctrl0, HIGH); //
    sel = 0;
    left_ros.writeMicroseconds(left_pwmROS);
    right_ros.writeMicroseconds(right_pwmROS);

    // Serial.println("from PCA9685");
    // 74ls157 will take pwm from pca9685
  }
  delay(50);

  /* // publish topic
    if (millis() > publisher_timer) {

    // publish battery and temperature
    control_msg.data[0] = left_pwm;
    control_msg.data[1] = right_pwm;
    control_msg.data[2] = left_pwm_ros;
    control_msg.data[3] = right_pwm_ros;
    control_msg.data[4] = sel;
    pub.publish(&control_msg);

    publisher_timer = millis() + DT;

    } */
  nh.spinOnce();

}

void conv_differential(int linear_x_center, int angular_z) {
  int insensitive_value = 50;
  if (linear_x_center > (neutral_pwm + insensitive_value) ||
      angular_z > (neutral_pwm + insensitive_value) ||
      linear_x_center < (neutral_pwm - insensitive_value) ||
      angular_z < (neutral_pwm - insensitive_value)) {
  }

  int angular_offset = insensitive(angular_z) - neutral_pwm;
  left_pwmRC = linear_x_center - angular_offset;
  right_pwmRC = linear_x_center + angular_offset;
  left_pwmRC = insensitive(left_pwmRC);
  right_pwmRC = insensitive(right_pwmRC);

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
