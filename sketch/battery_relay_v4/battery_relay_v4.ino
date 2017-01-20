/* ren ye
   2016-10-12
   rosserial for battery relay and temperature water cooler
   2016-10-20
   rosserial for cmd_vel; mux with tr324
   2017-01-13
   rosserial for battery relay, led indicator and e-stop
   2017-01-20
   separate physical e-stop and remote e-stop

   Connections:

   Arduino   <=> B relay1
   V12       <-> A1
   D5        <-> B1
   ~~~~~~~~~~<=> B relay2
   V12       <-> A2
   D6        <-> B2
   ~~~~~~~~~~<=> E relay
   D12       <-> Esense
   D2        <-> EB
   ~~~~~~~~~~<=> Battery sense
   A0        <-> Bsense1
   A1        <-> Bsense2
   ~~~~~~~~~~<=> LED shield
   Vin       <-> +ve
   D10       <-> Red
   D8        <-> Yellow
   D7        <-> Green
   ~~~~~~~~~~<=> Mega (for LED control signal)
   D4        <-> LED_msb
   D3        <-> LED_lsb




*/

#include <ros.h>
#include <Arduino.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>

// set up ros nodes and publishers
std_msgs::Float32MultiArray control_msg;
ros::Publisher pub("control", &control_msg);
ros::NodeHandle nh;

// initialize constants for battery relay and e stop relay
const float BR0[2] = {200.0, 100.0}; // voltage divider for battery 1
const float BR1[2] = {200.0, 100.0}; // voltage divider for battery 2
const float Vmin[2] = {11.4, 11.4}; // cutoff voltage for battery
const float ER[2] = {200.0, 100.0}; // voltage divider for e stop
const int DT = 1000; // 1 second

// digital pins
#define BRelay1 5  // battery1 relay switch, active high
#define BRelay2 6  // battery2 relay switch, active high
#define ERelay 2   // e stop relay switch, active high
#define LED_red 10 // led indicator red, active low
#define LED_yellow 8 // led indicator yellow, active low
#define LED_green 7 // led indicator gree, active low
#define LED_ctrl1 4 // led indicator control MSb
#define LED_ctrl0 3 // led indicator control LSb

// analog pins
#define Esense 12 // e stop voltage sense, HI~Open LO~Close
#define Bsense1 A1 // battery1 voltage sense
#define Bsense2 A0 // battery2 voltage sense

void setup()
{
  pinMode(BRelay1, OUTPUT);
  pinMode(BRelay2, OUTPUT);
  pinMode(ERelay, OUTPUT);
  pinMode(LED_red, OUTPUT);
  pinMode(LED_yellow, OUTPUT);
  pinMode(LED_green, OUTPUT);
  pinMode(LED_ctrl1, INPUT);
  pinMode(LED_ctrl0, INPUT);
  pinMode(Esense, INPUT);
  Serial.begin(57600);
  // reset led
  digitalWrite(LED_red, HIGH);
  digitalWrite(LED_yellow, HIGH);
  digitalWrite(LED_green, HIGH);
  delay(1000);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_yellow, LOW);
  digitalWrite(LED_green, LOW);
  // init battery relay
  digitalWrite(BRelay1, LOW);
  digitalWrite(BRelay2, LOW);
  // init e-stop relay
  digitalWrite(ERelay, LOW);
  // delay(1000);
  // digitalWrite(ERelay, HIGH);

  // init rosserial node and msg
  nh.initNode();
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
  int m_e_stop;
  float temp;
  int batt_status = 0;
  int led_status = 0;
  int led_read[2];

  // read physical e stop button data
  m_e_stop = digitalRead(Esense);
  // read led control data
  led_read[1] = digitalRead(LED_ctrl1);
  led_read[0] = digitalRead(LED_ctrl0);
  /*Serial.print("e stop sense ");
  Serial.print(batt_v[2]);
  Serial.print(", led ");
  Serial.print(digitalRead(LED_ctrl1));
  Serial.println(digitalRead(LED_ctrl0));*/
  // check and switch battery relay
  if (m_e_stop == LOW) { // stop mode by physical e-stop
    // Serial.println("case0");
    // E stop is engaged, power to thrusters cut off
    led_status = 0;
    digitalWrite(LED_red, HIGH ); // turn on red light only
    digitalWrite(LED_yellow, LOW);
    digitalWrite(LED_green, LOW);
    digitalWrite(ERelay, LOW);
  } else {
    if (led_read[1] == HIGH && led_read[0] == HIGH) { // stop mode by remote e-stop
      // Serial.println("case1");
      // E stop is engaged, power to thrusters cut off
      led_status = 0;
      digitalWrite(LED_red, HIGH); // turn on red light only
      digitalWrite(LED_yellow, LOW);
      digitalWrite(LED_green, LOW);
      digitalWrite(ERelay, LOW);  // open the relay
    } else if (led_read[1] == HIGH && led_read[0] == LOW) { //manual mode
      // Serial.println("case2");
      if (m_e_stop == LOW) { // stop mode by physical e-stop
        // Serial.println("case21");
        // E stop is engaged, power to thrusters cut off
        led_status = 0;
        digitalWrite(LED_red, HIGH); // turn on red light only
        digitalWrite(LED_yellow, LOW);
        digitalWrite(LED_green, LOW);
        digitalWrite(ERelay, LOW);
      } else {
        // Serial.println("case22");
        led_status = 1;
        digitalWrite(LED_red, LOW);
        digitalWrite(LED_yellow, HIGH);  // turn on yellow light only
        digitalWrite(LED_green, LOW);
        digitalWrite(ERelay, HIGH);
      }
    } else if (led_read[1] == LOW && led_read[0] == HIGH ) {  // automode
      // Serial.println("case3");
      if (m_e_stop == LOW) { // stop mode by physical e-stop
        // Serial.println("case31");
        // E stop is engaged, power to thrusters cut off
        led_status = 0;
        digitalWrite(LED_red, HIGH); // turn on red light only
        digitalWrite(LED_yellow, LOW);
        digitalWrite(LED_green, LOW);
        digitalWrite(ERelay, LOW);
      } else {
        // Serial.println("case32");
        led_status = 2;
        digitalWrite(LED_red, LOW);
        digitalWrite(LED_yellow, LOW);
        digitalWrite(LED_green, HIGH); // turn on green light only
        digitalWrite(ERelay, HIGH);
      }
    } else { // RC connection lost
      // Serial.println("case4");
      if (m_e_stop == LOW) { // stop mode by physical e-stop
        // Serial.println("case41");
        // E stop is engaged, power to thrusters cut off
        led_status = 0;
        digitalWrite(LED_red, HIGH); // turn on red light only
        digitalWrite(LED_yellow, LOW);
        digitalWrite(LED_green, LOW);
        digitalWrite(ERelay, LOW);
      } else {
        // Serial.println("case42");
        led_status = 3;
        digitalWrite(LED_red, LOW);
        digitalWrite(LED_yellow, HIGH); // turn on both yellow and green
        digitalWrite(LED_green, HIGH); 
        digitalWrite(ERelay, HIGH);
      }
    }
  }
  // read voltage divider data
  batt_v[0] = read_voltage(Bsense1);
  batt_v[1] = read_voltage(Bsense2);
  Serial.print("batt sense ");
  Serial.print(batt_v[0]);
  Serial.print(", ");
  Serial.println(batt_v[1]);
  Serial.print("relay status ");
 
  Serial.println(batt_v[0] - Vmin[0]);
  Serial.println(batt_v[1] - Vmin[1]);
  
  // check and switch battery relay
  if (batt_v[0] >= Vmin[0] && batt_v[1] >= Vmin[1]) {
    batt_status = 0;  // Batt1 healthy Batt2 healthy
     Serial.println(batt_status);
    digitalWrite(BRelay1, LOW);
    digitalWrite(BRelay2, LOW);
  } else if (batt_v[0] >= Vmin[0] && batt_v[1] < Vmin[1]) {
    batt_status = 1; // Batt1 healthy Batt2 unhealthy
     Serial.println(batt_status);
    digitalWrite(BRelay1, LOW);
    digitalWrite(BRelay2, HIGH);
  } else if (batt_v[0] < Vmin[0] && batt_v[1] >= Vmin[1]) {
    batt_status = 2; // Batt1 unhealthy Batt2 healthy
     Serial.println(batt_status);
    digitalWrite(BRelay1, HIGH);
    digitalWrite(BRelay2, LOW);
  } else {
    batt_status = 3; // Batt1 unhealthy Batt2 unhealthy
     Serial.println(batt_status);
    digitalWrite(BRelay1, LOW);
    digitalWrite(BRelay2, LOW);
    // blink once
    digitalWrite(LED_red, LOW);
    digitalWrite(LED_yellow, LOW);
    digitalWrite(LED_green, LOW);
    delay(1000);
    digitalWrite(LED_red, HIGH);
    digitalWrite(LED_yellow, HIGH);
    digitalWrite(LED_green, HIGH);
    delay(1000);
  }

  delay(100);

  // publish topic
  if (millis() > publisher_timer) {

    // publish battery and temperature
    control_msg.data[0] = batt_v[0];
    control_msg.data[1] = batt_v[1];
    control_msg.data[2] = batt_v[2];
    control_msg.data[3] = batt_status;
    control_msg.data[4] = led_status;
    control_msg.data[5] = 0; // reserved
    pub.publish(&control_msg);

    publisher_timer = millis() + DT;

  }
  nh.spinOnce();
}

float read_voltage(int id) {
  if (id == 1) {
    return (BR0[0] + BR0[1]) / BR0[1] * analogRead(id) * 5.0 / 1024;
  } else if (id == 0) {
    return (BR1[0] + BR1[1]) / BR1[1] * analogRead(id) * 5.0 / 1024;
  } else { // id == 2, e stop
    return (ER[0] + ER[1]) / ER[1] * analogRead(id) * 5.0 / 1024;
  }
}
