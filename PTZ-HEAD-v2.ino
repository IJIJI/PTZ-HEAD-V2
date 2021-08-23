#include "Arduino.h"
#include <EEPROM.h>

#include "TeensyStep.h"


#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>

#include "commands.h"



#define waitTimeRF 1000

#define fan0Pin PC9
#define fan1Pin PA8

// Define pin connections & motor's steps per revolution
#define enXPin PC11
#define enYPin PE3

#define stepsPerRevolutionX 200*6*4
#define stepsPerRevolutionY 200*6*4


// Stepper motorX(PA0, PC15);       // STEP pin: PA0, DIR pin: PC15
// Stepper motorY(PE5, PE6);       // STEP pin: PE5, DIR pin: PE6
Stepper* motors[] = { new stepper(PA0, PC15), new stepper(PE5, PE6) }; // STEPx pin: PA0, DIRx pin: PC15 STEPy pin: PE5, DIRy pin: PE6
StepControl controllerStep;
RotateControl controllerRotate0;
RotateControl controllerRotate1;


uint8_t camNum = 1;

enum headModeEnum {
  halted,
  standby,
  movePos,
  moveJoy,
  error
};


struct headModeStruct{
  headModeEnum mode = disabled;
  headModeEnum modeLast = disabled;
  uint8_t lastCommand;
  uint8_t prevLastCommand;
} currentMode;


struct motorData {
  uint8_t speedX;
  uint8_t speedY;
  uint8_t speedZ;
} motorSpeeds;

unsigned long lastRecieveRS = 0;
unsigned long lastRecieveRF = 0;

void setup()
{

  Serial.begin(115200);

  motors[0]
    .setAcceleration(1000)
    .setMaxSpeed(15000);
  motors[1]
    .setAcceleration(500)
    .setMaxSpeed(7500);

  // Declare pins as Outputs
  pinMode(enXPin, OUTPUT);
  pinMode(enYPin, OUTPUT);

  pinMode(fan0Pin, OUTPUT);
  pinMode(fan1Pin, OUTPUT);

  delay(500);

  currentMode.mode = standby;

  digitalWrite(enXPin, LOW);
  digitalWrite(enYPin, LOW);
  digitalWrite(fan0Pin, HIGH);
  digitalWrite(fan1Pin, HIGH);
  // analogWrite(fan0Pin, 255); //TODO fix analogwrite
  // analogWrite(fan1Pin, 255); //TODO fix analogwrite


  delay(1000);
}
void loop()
{

  // if (Serial.available()){
  //   char inData[15];
  //   Serial.readBytesUntil(0xFF, inData, 14);
  // }

  uint8_t inDataRS = NULL;
  uint8_t inDataRF = NULL;

  if (inDataRS = receiveCommandRS()){
    lastRecieveRS = millis();
  }

  if (inDataRF = receiveCommandRF()){
    lastRecieveRF = millis();
  }

// !

  if (currentMode.mode == error){

    if(currentMode.modeLast != currentMode.mode){
      controllerStep.emergencyStop();
      controllerRotate.emergencyStop();

      delay(200);

      digitalWrite(enXPin, HIGH);
      digitalWrite(enYPin, HIGH);
      digitalWrite(fan0Pin, LOW);
      digitalWrite(fan1Pin, LOW);
      currentMode.modeLast = currentMode.mode;
    }

  }

// ?

  else if (currentMode.mode == standby || currentMode.mode == moveJoy || currentMode.mode == movePos){

    if(currentMode.modeLast != currentMode.mode){
      digitalWrite(enXPin, LOW);
      digitalWrite(enYPin, LOW);
      digitalWrite(fan0Pin, HIGH);
      digitalWrite(fan1Pin, HIGH);
      currentMode.modeLast = currentMode.mode;
    }

  
    if(inDataRS){
      if (checkSumCheck(inDataRS) && inDataRS[0] == camNum){
        handleInData(inDataRS);
        lastRecieveRS = millis();
      }
    }
    else if (inDataRF && lastRecieveRS + waitTimeRF < millis()){

      if (checkSumCheck(inDataRF) && inDataRF[0] == camNum){
        handleInData(inDataRF);
        lastRecieveRF = millis();
      }
      
    }



    if (currentMode.lastCommand == moveJoy){
      
      controllerRotate0.rotateAsync(motors[0]);
      controllerRotate1.rotateAsync(motors[1]);

      controllerRotate0.overrideSpeed(map(motorSpeeds.speedX, 1, 254, -1, 1));
      controllerRotate1.overrideSpeed(map(motorSpeeds.speedY, 1, 254, -1, 1));

    }

    if (currentMode.prevLastCommand == moveJoy && currentMode.lastCommand != moveJoy && currentMode.lastCommand != movePos){
      motors[0].setTargetAbs(motors[0].getPosition());
      motors[1].setTargetAbs(motors[1].getPosition());
    }

    if (currentMode.lastCommand == movePos){
      controllerStep.moveAsync(motors);
    }

  }

// ?

  else if (currentMode.mode == halted ){

    if(currentMode.modeLast != currentMode.mode){
      controllerStep.stopAsync();
      controllerRotate.stopAsync();
      currentMode.modeLast = currentMode.mode;
    }

  }




  // if (currentMode.mode == disabled ){

  //   if(currentMode.modeLast != disabled){
  //     digitalWrite(enXPin, HIGH);
  //     digitalWrite(enYPin, HIGH);
  //     digitalWrite(fan0Pin, LOW);
  //     digitalWrite(fan1Pin, LOW);
  //     currentMode.modeLast = currentMode.mode;
  //   }



  // }
  // else if (currentMode.mode == standby || currentMode.mode == movePos || currentMode.mode == moveJoy){

  //   if(currentMode.modeLast != standby && currentMode.modeLast != movePos && currentMode.modeLast != moveJoy){
  //     digitalWrite(enXPin, LOW);
  //     digitalWrite(enYPin, LOW);
  //     digitalWrite(fan0Pin, HIGH);
  //     digitalWrite(fan1Pin, HIGH);
  //     currentMode.modeLast = currentMode.mode;
  //   }

  //   if(inDataRS){
  //     if (checkSumCheck(inDataRS) && inDataRS[0] == camNum){
  //       handleInData(inDataRS);
  //       lastRecieveRS = millis();
  //     }
  //   }
  //   else if (inDataRF && lastRecieveRS + waitTimeRF < millis()){

  //     if (checkSumCheck(inDataRF) && inDataRF[0] == camNum){
  //       handleInData(inDataRF);
  //       lastRecieveRF = millis();
  //     }
      
  //   }


  // }
  // else if (currentMode.mode == error){

  // }





  // if (Serial.available()) {

  //   uint8_t inData[14];

  //   inData = Serial.readBytesUntil(0xFF);
  //   lastRecieve = millis();
  // }


  if (lastRecieveRS + 150 > millis()){
    digitalWrite(fan1Pin, HIGH);
  }
  else{
    digitalWrite(fan1Pin, LOW);
  }


}



//? RS485 Protocol


uint8_t receiveCommandRS(){
  if (Serial.available()) {
    uint8_t inData[14];
    Serial.readBytesUntil(0xFF, inData, 15);
    return inData;
  }
  return NULL;
}

uint8_t receiveCommandRF(){

  return NULL;
}

bool checkSumCheck(uint8_t inData[]){

  uint16_t checkSum = 0;
  for(int x; x < 12; x++){
    checkSum += inData[x];
  }
  checkSum = checkSum % 256;

  if (inData[12] == checkSum){
    return true;
  }

}

void handleInData(uint8_t inData[]){

  currentMode.prevLastCommand = currentMode.lastCommand;
  currentMode.lastCommand = inData[1];
  switch (inData[1])
  {
  case 1:
    joyUpdate(inData[2], inData[3], inData[4], inData[5]);
    break;
  
  case 4:
    writePos(inData[2]);
    break;
  
  case 6:
    callPos(inData[2]);
    break;
  
  case 7:
    setCoords(inData[2], inData[3], inData[4], inData[5], inData[6], inData[7], inData[8], inData[9]);
    break;
  
  case 8:
    callError(inData[2]);
    break;

  }

}

//? command functions

void joyUpdate(uint8_t joyX, uint8_t joyY, uint8_t joyZ, uint8_t joyA){

  currentMode.mode = moveJoy;

  motorSpeeds.speedX = joyX;
  motorSpeeds.speedY = joyY;
  motorSpeeds.speedX = joyZ;

}

void writePos(uint8_t posNum){

  currentMode.mode = movePos;

}

void callPos(uint8_t posNum){

}

void setCoords(uint8_t xPosA, uint8_t xPosB, uint8_t yPosA, uint8_t yPosB, uint8_t zPosA, uint8_t zPosB, uint8_t aPosA, uint8_t aPosB){

}

void callError(uint8_t errorNum){

}