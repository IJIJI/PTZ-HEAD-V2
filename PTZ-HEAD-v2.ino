#include "Arduino.h"
#include <EEPROM.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

// #include <SPI.h>
// #include <RF24.h>

#include "commands.h"



#define waitTimeRF 1000

#define fan0Pin PC9
#define fan1Pin PA8
#define fan2Pin PC8
#define fan3Pin PC7

// Define pin connections & motor's steps per revolution
#define enXPin PC11
#define enYPin PE3

#define stepsPerRevolutionX 200*6*4
#define stepsPerRevolutionY 200*6*4


// ! Teensystep remainder
// Stepper motorX(PA0, PC15);       // STEP pin: PA0, DIR pin: PC15
// Stepper motorY(PE5, PE6);       // STEP pin: PE5, DIR pin: PE6
// Stepper* motors[] = { new Stepper(PA0, PC15), new Stepper(PE5, PE6) }; // STEPx pin: PA0, DIRx pin: PC15 STEPy pin: PE5, DIRy pin: PE6

// StepControl controllerStep;
// RotateControl controllerRotate0;
// RotateControl controllerRotate1;
// !

AccelStepper xAxis(1, PA0, PC15);
AccelStepper yAxis(1, PE5, PE6);

uint8_t camNum = 1;

enum headModeEnum {
  halted,
  standby,
  movePos,
  moveJoy,
  error
};


struct headModeStruct{
  headModeEnum mode = halted;
  headModeEnum modeLast = halted;
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

// HardwareSerial Serial1(PIN_A10, PIN_A9);

void setup()
{

  Serial1.begin(115200);
  Serial.begin(115200);


  xAxis.setMaxSpeed(5000);
  // xAxis.setMaxSpeed(750);
  xAxis.setAcceleration(4000);

  yAxis.setMaxSpeed(2500);
  // xAxis.setMaxSpeed(750);
  yAxis.setAcceleration(2000);



  // Declare pins as Outputs
  pinMode(enXPin, OUTPUT);
  pinMode(enYPin, OUTPUT);

  pinMode(fan0Pin, OUTPUT);
  pinMode(fan1Pin, OUTPUT);
  pinMode(fan2Pin, OUTPUT);
  pinMode(fan3Pin, OUTPUT);

  delay(500);

  currentMode.mode = standby;

  // digitalWrite(enXPin, LOW);
  // digitalWrite(enYPin, LOW);
  digitalWrite(fan0Pin, HIGH);
  digitalWrite(fan1Pin, HIGH);
  // analogWrite(fan0Pin, 255); //TODO fix analogwrite
  // analogWrite(fan1Pin, 255); //TODO fix analogwrite


  delay(1000);
}
void loop()
{


  uint8_t inDataRS[14] = {NULL};
  uint8_t inDataRF[14] = {NULL};


  if (Serial1.available()) {
    Serial1.readBytesUntil(0xFF, inDataRS, 14);
    Serial.write(inDataRS, 14);
    Serial1.flush();
  }

  if (Serial1.available() && false) {
    // Serial1.readBytesUntil(0xFF, inDataRS, 14);
    // Serial1.flush();

    uint8_t writeNum = 0;
    uint8_t lastRecVal = 0;
    while (lastRecVal != 255 && writeNum < 15){
      lastRecVal = Serial1.read();
      inDataRS[writeNum] = lastRecVal;
      writeNum++;
      delayMicroseconds(10);
      digitalWrite(fan3Pin, HIGH);
      Serial.write(lastRecVal);
      
    }
    Serial.write(inDataRS, 14);
    // Serial1.flush();
    digitalWrite(fan3Pin, LOW);
  }



// !

  if (currentMode.mode == error){

    if(currentMode.modeLast != currentMode.mode){
      xAxis.stop();
      yAxis.stop();

      delay(200);

      digitalWrite(enXPin, HIGH);
      digitalWrite(enYPin, HIGH);
      // digitalWrite(fan0Pin, LOW);
      // digitalWrite(fan1Pin, LOW); //TODO COmment back in!
      currentMode.modeLast = currentMode.mode;
    }

  }

// ?

  else if (currentMode.mode == standby || currentMode.mode == moveJoy || currentMode.mode == movePos){

    if(currentMode.modeLast != currentMode.mode && false){
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
      else{
        digitalWrite(fan2Pin, LOW);
      }
      
    }
    else if (inDataRF && lastRecieveRS + waitTimeRF < millis()){

      if (checkSumCheck(inDataRF) && inDataRF[0] == camNum){
        handleInData(inDataRF);
        lastRecieveRF = millis();
      }
      
    }




    if (currentMode.mode == moveJoy){
      
      // controllerRotate0.rotateAsync(motorX);
      // controllerRotate1.rotateAsync(motorY);

      // controllerRotate0.overrideSpeed(map(motorSpeeds.speedX, 1, 254, -1, 1));
      // controllerRotate1.overrideSpeed(map(motorSpeeds.speedY, 1, 254, -1, 1));

      if (motorSpeeds.speedX == 127){
        xAxis.setSpeed(0);
      }
      else{
        xAxis.setAcceleration(5000000);
        xAxis.setSpeed(map(motorSpeeds.speedX, 1, 254, -1500, 1500));
      }

      if (motorSpeeds.speedY == 127){
        yAxis.setSpeed(0);
      }
      else{
        yAxis.setAcceleration(5000000);
        yAxis.setSpeed(map(motorSpeeds.speedY, 1, 254, -1500, 1500));
      }

      xAxis.runSpeed();
      yAxis.runSpeed();


    }

    if (currentMode.prevLastCommand == moveJoy && currentMode.lastCommand != moveJoy && currentMode.lastCommand != movePos){
      xAxis.moveTo(xAxis.currentPosition());
      yAxis.moveTo(yAxis.currentPosition());
    }

    if (currentMode.mode == movePos){
      xAxis.run();
      yAxis.run();
    }

    // if (lastRecieveRS + 150 > millis() && false){
    //   digitalWrite(fan1Pin, HIGH);
    // }
    // else{
    //   digitalWrite(fan1Pin, LOW);
    // }


  }

// ?

  else if (currentMode.mode == halted ){

    if(currentMode.modeLast != currentMode.mode){
      xAxis.stop();
      yAxis.stop();
      currentMode.modeLast = currentMode.mode;
    }

  }

  if (currentMode.mode == moveJoy){
    digitalWrite(fan1Pin, HIGH);

  }
  else{
    digitalWrite(fan1Pin, LOW);
  }





}



//? RS485 Protocol


uint8_t* receiveCommandRS(){
  if (Serial1.available()) {
    uint8_t* inData = new uint8_t[14];
    Serial1.readBytesUntil(0xFF, inData, 15);
    return inData;
  }
  return NULL;
}

uint8_t* receiveCommandRF(){

  return NULL;
}

// bool checkSumCheck(uint8_t inData[]){

//   uint16_t checkSum = 0;
//   for(int x = 0; x < 12; x++){
//     if (inData[x] >= 255){
//       return false;
//     }
//     checkSum += inData[x];
//   }
//   checkSum = checkSum % 256;

//   if (checkSum >= 255){
//     checkSum = 254;
//   }

//   if (inData[12] == checkSum && true){
//     return true;
//   }

// }

bool checkSumCheck(uint8_t inData[]){

  // return true;

  uint16_t checkSum = 0;
  for(int x = 0; x < 12; x++){
    // if (inData[x] >= 255){
    //   Serial1.flush();
    //   return false;
    // }
    checkSum += inData[x];
  }
  checkSum = checkSum % 256;

  if (checkSum >= 255){
    checkSum = 254;
  }

  if (inData[12] == checkSum && true){
    return true;
  }
  else{
    return false;
  }

}

void handleInData(uint8_t inData[]){

  if (inData[0] <= 10 && inData[0] > 0) {
    digitalWrite(fan2Pin, HIGH);
  }

  

  currentMode.prevLastCommand = currentMode.lastCommand;
  currentMode.lastCommand = inData[1];
  switch (inData[1])
  {
  case 1:
    currentMode.mode = moveJoy;
    joyUpdate(inData[2], inData[3], inData[4], inData[5]);
    break;
  
  case 4:
    writePos(inData[2]);
    break;
  
  case 6:
    currentMode.mode = movePos;
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

  

  motorSpeeds.speedX = joyX;
  motorSpeeds.speedY = joyY;
  motorSpeeds.speedZ = joyZ;

}

void writePos(uint8_t posNum){

  currentMode.mode = movePos;

}

void callPos(uint8_t posNum){

  if (posNum == 5){
    currentMode.mode = error;
  }
  
}

void setCoords(uint8_t xPosA, uint8_t xPosB, uint8_t yPosA, uint8_t yPosB, uint8_t zPosA, uint8_t zPosB, uint8_t aPosA, uint8_t aPosB){

}

void callError(uint8_t errorNum){

}