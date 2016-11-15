// in ROS:
// 1. run roscore
// 2. in a new tab(ctrl+shift+t) 
//    rosrun rosserial_python serial_node.py /dev/ttyACM0 _baud:=57600 
// 3. in another tab

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define estopPin 6

int input;

ros::NodeHandle nh;

byte relayPin[4] = {
  2,7,8,10};
//D2 -> RELAY1
//D7 -> RELAY2
//D8 -> RELAY3
//D10 -> RELAY

void callBack(const geometry_msgs::Vector3 && msg){
  input=int(msg.z);
}

ros::Subscriber<geometry_msgs::Vector3>sub("/chatter", &callBack);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  
  for(int i = 0; i < 4; i++)  pinMode(relayPin[i],OUTPUT);
  Serial.begin(57600); 
  delay(100);
  for(int j = 0; j < 4; j++)  digitalWrite(relayPin[j],LOW);
}

void loop() {
    if(digitalRead(estopPin)==HIGH){    
        Serial.println("1");
        digitalWrite(relayPin[0],HIGH);
        digitalWrite(relayPin[1],LOW);
        digitalWrite(relayPin[2],LOW);
    }
    else{
      if (input==2){ //control by RC
        Serial.println("Relay2");
        digitalWrite(relayPin[0],LOW);
        digitalWrite(relayPin[1],HIGH);
        digitalWrite(relayPin[2],LOW);
      }
      else if (input==3){ //automation
        Serial.println("Relay3");
        digitalWrite(relayPin[0],LOW);
        digitalWrite(relayPin[1],LOW);
        digitalWrite(relayPin[2],HIGH);
      }
      else{
        Serial.println("No Relay");
        digitalWrite(relayPin[0],LOW);
        digitalWrite(relayPin[1],LOW);
        digitalWrite(relayPin[2],LOW);
      }
    }
  nh.spinOnce();
  delay(100);
}

