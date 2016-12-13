
#define estopPin 9
#define modePin 12

byte relayPin[4] = {2,7,8,10};
//D2 -> RELAY1
//D7 -> RELAY2
//D8 -> RELAY3
//D10 -> RELAY4

void setup(){

  pinMode(estopPin, INPUT);
  pinMode(modePin, INPUT);
  
  for(int i = 0; i < 4; i++)  pinMode(relayPin[i],OUTPUT);
  for(int j = 0; j < 4; j++)  digitalWrite(relayPin[j],LOW);
  
  Serial.begin(57600);
  delay(100);
}

void loop() {
  
    if(digitalRead(estopPin)==HIGH){                   
        Serial.println("Relay1");
        digitalWrite(relayPin[0],HIGH);
        digitalWrite(relayPin[1],LOW);
        digitalWrite(relayPin[2],LOW);
    }
   
    else{
      if (digitalRead(modePin)==HIGH){ //control by RC
        Serial.println("Relay2");
        digitalWrite(relayPin[0],LOW);
        digitalWrite(relayPin[1],HIGH);
        digitalWrite(relayPin[2],LOW);
       
      }
      else{ //automation
        Serial.println("Relay3");
        digitalWrite(relayPin[0],LOW);
        digitalWrite(relayPin[1],LOW);
        digitalWrite(relayPin[2],HIGH);
       
      }
      
    }

  delay(100);
}
