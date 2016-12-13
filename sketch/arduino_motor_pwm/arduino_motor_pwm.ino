#include <SPI.h>

#define pwr5volt 6  
#define slaveSelectPin 53
#define RC2 8
#define RC3 9
#define RC4 7
#define modePin 3
#define estopPin 1

#define throttle_input 11
#define steering_input 12


int throttle=1500; // Here's where we'll keep our channel values
int steering=1500; 
int mode=1500;// 

int forward=1500;
int turn=1500;
int forwardROS=1500;
int turnROS=1500;

double right_calib=1;
double left_calib=1;



void setup() {

    SPI.begin(); //initialize SPI
    pinMode(pwr5volt,OUTPUT);
    pinMode(slaveSelectPin,OUTPUT);
    digitalWrite(pwr5volt,HIGH);

    pinMode(RC2, INPUT); // Set our input pins as such
    pinMode(RC3, INPUT);
    pinMode(RC4, INPUT);

    pinMode(throttle_input, INPUT);
    pinMode(steering_input, INPUT);

    pinMode(modePin, OUTPUT);
    pinMode(estopPin, OUTPUT);
	
    //unicycleRun(0, 0);  // reset to neutral

    Serial.begin(57600); // Pour a bowl of Serial
}

void loop() {

    throttle = pulseIn(RC2, HIGH, 25000); // Read the pulse width of 
    steering = pulseIn(RC3, HIGH, 25000); // each channel
    mode = pulseIn(RC4, HIGH, 25000); // 
    
    throttle=constrain(throttle, 1100, 1900);
    steering=constrain(steering, 1100, 1900);
    
    forwardROS  = pulseIn(throttle_input, HIGH, 25000);
    turnROS     = pulseIn(steering_input, HIGH, 25000);

    if (forwardROS==0){
      forwardROS=1500;
    }

    if (turnROS==0){
      turnROS=1500;
    }

    if((mode>1000) && (mode<=1500)){
        //manual mode
        forward   = map(throttle, 1100, 1900, -500, 500); //map values from RC
        turn      = map(steering, 1100, 1900,-500, 500);
        digitalWrite(modePin, HIGH);
    }
    else if((mode>1500) && (mode<2000)){
      
        //autonomous mode
        forward   = map(forwardROS, 1100, 1900, -500, 500); //value is -500 to 500
        turn      = map(turnROS, 1100, 1900, -500, 500);
        digitalWrite(modePin, LOW);
    }
    else{
        //e-stop switch
        forward=0;
        turn=0;
        digitalWrite(estopPin, HIGH);
    }
    

    // constrain in case exceeds
    //forward = constrain(forward, -500, 500);
    //turn = constrain(turn, -500, 500);

    unicycleRun(forward, turn);
    printValue();
    delay(100);

}

void unicycleRun(int forward, int turn){
    int Ur;
    int Ul;
    int digitalStepRight;
    int digitalStepLeft;
    int Ur_out;
    int Ul_out;

    Ur=right_calib*(forward-turn/2); //here turning in cw is positive250
    Ul=left_calib*(forward+turn/2); //750
    //Ur & Ul ranges from -750 to 750 if right and left calibration factors = 1 
    Ur=-Ur;
    Ul=-Ul;
    Ur=constrain(Ur, -750, 750);
    Ul=constrain(Ul, -750, 750);

    //map Ur and Ul to digital potentiometer values 0-255, map back to 1000-2000
    Ur_out=map(Ur, -750, 750, 1000, 2000);
    Ul_out=map(Ul, -750, 750, 1000, 2000);

    digitalStepRight=mapToResistance(Ur_out);
    digitalStepLeft=mapToResistance(Ul_out);

    Serial.print("\nright: ");
    Serial.print(digitalStepRight);

    Serial.print("\nleft:");
    Serial.print(digitalStepLeft);
    setMotor(digitalStepRight, digitalStepLeft);

}

int mapToResistance(int input){

    int digitalStep;

    if (input<=2000 && input>1550)
    {
        digitalStep = map(input,1551,2000, 80, 0); 
    }
    else if (input<=1550 && input>1450)
    {
        digitalStep =120;
    }
    else if (input<=1450&&input>=1000)
    {
        digitalStep = map(input,900,1450,255,180); 
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
    Serial.print("\nThrottle:"); // Print the value of 
    Serial.print(throttle);    // each channel

    Serial.print("\nSteering:");
    Serial.print(steering);

    Serial.print("\nMode:");
    Serial.print(mode);

    Serial.print("\nforwardRos:");
    Serial.print(forwardROS);

    Serial.print("\nturnROS:");
    Serial.print(turnROS);


    Serial.print("\n\n");

    delay(100); // I put this here just to make the terminal window happier  
}
