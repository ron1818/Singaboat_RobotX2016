
/* Given forward and turn commands(through ROS or RC), run motors accordingly
 * v1: Lim Jiehan
 * v2: Reinaldo 
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
 * connect Arduino to AD8402 digital potentiometer, Futaba receiver, embedded computer 
 * digital potentiometer connect to Minn Kota motor
 * 
 * motor cable connection sequence (black(B), green(A), grey(W swiper)) seen from upright AD8402 position
 * 
 *
 */

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#include <SPI.h>

#define pwr5volt 6  
#define slaveSelectPin 53
#define RC1 7
#define RC2 8
#define RC3 9

ros::NodeHandle nh;

int ch1=1500; // Here's where we'll keep our channel values
int ch2=1500; 
int ch3=1500;// channel 3 unused, can be used for switching between ROS command or RC command
int forward;
int turn;
int forwardROS;
int turnROS;
double right_calib=1.0;
double left_calib=1.0;

geometry_msgs::Vector3 movement; //for chatter

void changeVar(const geometry_msgs::Twist& input){
  forwardROS=input.linear.x;
  turnROS=input.angular.z;

  //updates calibration factor
  right_calib=input.linear.y;
  left_calib=input.linear.z; 
}

ros::Subscriber<geometry_msgs::Twist>sub("/motor_val", &changeVar);

ros::Publisher chatter("chatter", &movement);

void setup() {
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  
  SPI.begin(); //initialize SPI
  pinMode(pwr5volt,OUTPUT);
  pinMode(slaveSelectPin,OUTPUT);
  digitalWrite(pwr5volt,HIGH);

  pinMode(RC1, INPUT); // Set our input pins as such
  pinMode(RC2, INPUT);
  pinMode(RC3, INPUT);

  Serial.begin(57600); // Pour a bowl of Serial

}

void loop() {
  
  ch1 = pulseIn(RC1, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(RC2, HIGH, 25000); // each channel
  ch3 = pulseIn(RC3, HIGH, 25000); // ch3 unused at the moment
  

  if(ch3<1300){
    //input from RC
    forward = map(ch1,1100,1900,-500,500); //map values from RC
    turn = map(ch2,1100,1900,-500,500);
  }
  else{
  //input from ROS publisher, when ch3 at stable position, input taken from turtlebot_teleop
    forward=forwardROS; //value is -500 to 500
    turn=turnROS;
  }


  unicycleRun(forward, turn);
  
  printValue();
  nh.spinOnce();
  delay(100);
  
}

void unicycleRun(int forward, int turn){
  int Ur;
  int Ul;
  int digitalStepRight;
  int digitalStepLeft;
  
  Ur=right_calib*(forward-turn/2); //here turning in cw is positive
  Ul=left_calib*(forward+turn/2);
  //Ur & Ul ranges from -750 to 750 if right and left calibration factors = 1 
  Ur=-Ur;
  Ul=-Ul;
  
  //map Ur and Ul to digital potentiometer values 0-255, map back to 1000-2000
  Ur=map(Ur, -750, 750, 1000, 2000);
  Ul=map(Ul, -750, 750, 1000, 2000);

  digitalStepRight=mapToResistance(Ur);
  digitalStepLeft=mapToResistance(Ul);
  
  setMotor(digitalStepRight, digitalStepLeft);
  
  movement.x=digitalStepRight; //0=>fastest forward, 255 =>fastest backwards, 120 =>stops
  movement.y=digitalStepLeft;
  chatter.publish(&movement);
}

int mapToResistance(int input){
  
  int digitalStep;
  
  if (input<=2000&&input>1550)
  {
    digitalStep = map(input,1551,2000,90,0); //right (80-50) left(80-0)
  }
  else if (input<=1550&&input>1450)
  {
    digitalStep =120;
  }
  else if (input<=1450&&input>=1000)
  {
    digitalStep = map(input,1000,1450,256,205); 
  }
  
  return digitalStep;
}

void setMotor(int Right, int Left)
{
    digitalWrite(slaveSelectPin,LOW);//take SS pin to low to select the chip
    SPI.transfer(0);//adress of the first potentiometer 00 (motor select)
    SPI.transfer(Right);//second data is digital value
    digitalWrite(slaveSelectPin,HIGH);//take SS pin high to deselect chip

    delay(50);//wait
    
    digitalWrite(slaveSelectPin,LOW);
    SPI.transfer(1); //adress of second potentiometer 01
    SPI.transfer(Left);
    digitalWrite(slaveSelectPin,HIGH);
}

void printValue()
{ 

  //information to serial monitor
  Serial.print("Ch1:"); // Print the value of 
  Serial.print(ch1);        // each channel

  Serial.print("   Ch2:");
  Serial.print(ch2);

  Serial.print("   Ch3:");
  Serial.print(ch3);

  delay(100); // I put this here just to make the terminal window happier  
}
