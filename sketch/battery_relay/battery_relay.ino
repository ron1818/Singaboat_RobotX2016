/* ren ye
   2016-10-12
   rosserial for battery relay and temperature water cooler
   2016-10-20
   rosserial for cmd_vel; mux with tr324

   Connections:
   TR324     <-> Arduino
   Channel 1 <-> D7
   Channel 2 <-> D8
   Ack       <-> D2 (interrupt)

   TR324     <-> alfro ESC
   +, -      <-> +, - (5v)

   Arduino   <-> alfro ESC
   D9~       <-> S1
   D10~      <-> S2

   Arduino   <-> RPi
   USB       <-> USB

   Arduino   <-> sensors
   D12       <-> pump relay switch
   D11       <-> battery relay switch
   A0        <-> temperature sig
   A1        <-> battery 1 volt
   A2        <-> battery 2 volt

*/

#include <ros.h>
// #include <ros/time.h>
#include <Arduino.h>
// #include <std_msgs/String.h>
#include <std_msgs/Float32.h>
// use uint8multiarray
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
// for esc, use servo library
#include <Servo.h>

Servo left_esc;
Servo right_esc;

// set up ros nodes and publishers
std_msgs::Float32MultiArray control_msg;
// std_msgs::Float32 pwm_ack_msg;


ros::Publisher pub("control", &control_msg);

ros::NodeHandle nh;

// initialize constants for battery relay and temp sensor
const float R0[2] = {200.0, 100.0}; // voltage divider for battery 1
const float R1[2] = {200.0, 100.0}; // voltage divider for battery 2
const float Vmin[2] = {11.4, 11.6}; // cutoff voltage for battery
const float temp_sw_on = 40; // switch on temperature

// const int BAUD = 115200;
const int DT = 1000; // 1 second

// digital pins
#define temp_sw 12
#define batt_sw 4
#define led_pin 13
#define left_thruster 9
#define right_thruster 10
#define tr324_ack 2
#define tr324_ch1 7
#define tr324_ch2 8

// analog pins
#define temp_pin A3
#define batt1_v_pin A1
#define batt2_v_pin A2

// pwm setup
const int max_pwm = 1900;
const int min_pwm = 1100;
const int neutral_pwm = 1500;
const float wheel_sep = 0.4;

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

void setup()
{
  //Serial.begin(BAUD);
  pinMode(temp_sw, OUTPUT);
  pinMode(batt_sw, OUTPUT);
  pinMode(led_pin, OUTPUT);
  pinMode(tr324_ack, INPUT);
  pinMode(tr324_ch1, INPUT);
  pinMode(tr324_ch2, INPUT);
  pinMode(left_thruster, OUTPUT);
  pinMode(right_thruster, OUTPUT);

  // attach esc
  left_esc.attach(left_thruster);
  right_esc.attach(right_thruster);

  left_esc.writeMicroseconds(neutral_pwm);
  right_esc.writeMicroseconds(neutral_pwm);

  // reset led
  digitalWrite(led_pin, LOW);
  // switch to primary battery
  digitalWrite(batt_sw, HIGH);

  nh.initNode();
  nh.subscribe(pwm_sub);

  control_msg.layout.dim = (std_msgs::MultiArrayDimension *)
                        malloc(sizeof(std_msgs::MultiArrayDimension) * 1);
  control_msg.layout.dim->label = "arduino_reading";
  control_msg.layout.dim->size = 6;
  // batt_msg.layout.dim_length = 1;
  control_msg.layout.data_offset = 0;
  control_msg.data = (float *)malloc(sizeof(float) * 5);
  control_msg.data_length = 6;

  nh.advertise(pub);
  delay(2000);

}

long action_timer;
long publisher_timer;

void loop() {
  float batt_v[2];
  float temp;
  int left_pwm = neutral_pwm;
  int right_pwm = neutral_pwm;
  int batt_status = 0;

  // read voltage divider data
  batt_v[0] = read_voltage(batt1_v_pin);
  batt_v[1] = read_voltage(batt2_v_pin);
  // check and switch
  if (batt_v[0] >= Vmin[0]) {
    batt_status = 0;
    digitalWrite(batt_sw, LOW); // switch to secondary battery
    // digitalWrite(batt_sw, LOW); // switch to primary battery
  } else if ((batt_v[0] < Vmin[0]) && (batt_v[1] >= Vmin[1])) {
    batt_status = 1;
    digitalWrite(batt_sw, HIGH); // switch to secondary battery
  } else if (batt_v[1] < Vmin[1]) {
    batt_status = 2;
    digitalWrite(batt_sw, HIGH); // switch to secondary battery
  }

  // read temperature data
  temp = read_temperature(temp_pin);
  if (temp > temp_sw_on) {
    digitalWrite(temp_sw, HIGH);
  } else {
    digitalWrite(temp_sw, LOW);
  }

  // read pwm to motor
  if (digitalRead(tr324_ack) == HIGH) {
    manual_control();
    left_pwm = left_pwmRC;
    right_pwm = right_pwmRC;
  } else {
    left_pwm = left_pwmROS;
    right_pwm = right_pwmROS;
  }

  // write to pwm motor pin

  left_esc.writeMicroseconds(left_pwm);
  right_esc.writeMicroseconds(right_pwm);

  delay(100);

  // publish topic
  if (millis() > publisher_timer) {

    // publish battery and temperature
    control_msg.data[0] = batt_v[0];
    control_msg.data[1] = batt_v[1];
    control_msg.data[2] = temp;
    // two more to get the pwm data from arduino, debug use
    control_msg.data[3] = left_pwm;
    control_msg.data[4] = right_pwm;
    control_msg.data[5] = batt_status;
    pub.publish(&control_msg);

    publisher_timer = millis() + DT;

  }
  nh.spinOnce();
}

float read_voltage(int id) {
  if (id == 1) {
    return (R0[0] + R0[1]) / R0[1] * analogRead(id) * 5.0 / 1024;
  } else {
    return (R1[0] + R1[1]) / R1[1] * analogRead(id) * 5.0 / 1024;
  }
}

float read_temperature(int temp_pin) {
  float voltage = analogRead(temp_pin) * 5.0 / 1024.0;  
  return (voltage - 0.5 ) * 100.0;
}

void manual_control(void) {
  int tmp_angular_z = pulseIn(tr324_ch2, HIGH, 25000);
  int tmp_linear_x_center = pulseIn(tr324_ch1, HIGH, 25000);
  if (tmp_linear_x_center > 500) {
    linear_x_center = tmp_linear_x_center;
  }
  if (tmp_angular_z > 500) {
    angular_z = tmp_angular_z;
  }

  conv_differential(linear_x_center, angular_z);
}

void conv_differential(int linear_x_center, int angular_z) {
  int insensitive_value = 50;
  if (linear_x_center > (neutral_pwm + insensitive_value) ||
      angular_z > (neutral_pwm + insensitive_value) ||
      linear_x_center < (neutral_pwm - insensitive_value) ||
      angular_z < (neutral_pwm - insensitive_value)) {
    digitalWrite(led_pin, HIGH);
  } else {
    digitalWrite(led_pin, LOW);
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

  /* if (pwm < (neutral_pwm + insensitive_value) ||
      pwm > (neutral_pwm - insensitive_value)) {
    return neutral_pwm;
  } else {
    return pwm;
  }*/
  return pwm;
  
}
