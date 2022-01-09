#include "Arduino.h"
#include <EEPROM.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

// #include <SPI.h>
// #include <RF24.h>

#include "commands.h"


// #define debug


#define waitTimeRF 1000

#define ledWaitTime 500

#define stepsPerRevolution 200*6

#define X_ACCELERATION 4000
#define Y_ACCELERATION 2000

#define X_MAX_SPEED 1200
#define Y_MAX_SPEED 800	

#define EEPROM_SIZE 512

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

struct vector {
  int16_t x;
  int16_t y;
  // int16_t z;
  // int16_t a;
};

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

struct inData {
  uint8_t data[15] = {NULL};
  unsigned long lastReceived = 0;
  bool checksum = false;
};

unsigned long lastRecieveRS = 0;
unsigned long lastRecieveRF = 0;

// HardwareSerial Serial1(PIN_A10, PIN_A9);

void setup()
{

  Serial1.begin(115200);
  Serial.begin(115200);


  xAxis.setMaxSpeed(X_MAX_SPEED);
  xAxis.setAcceleration(X_ACCELERATION);

  yAxis.setMaxSpeed(Y_MAX_SPEED);
  yAxis.setAcceleration(Y_ACCELERATION);



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
  digitalWrite(fan1Pin, HIGH); //TODO change fan pin names
  // analogWrite(fan0Pin, 255); //TODO fix analogwrite
  // analogWrite(fan1Pin, 255); //TODO fix analogwrite


  delay(1000);


  digitalWrite(fan1Pin, LOW);
}
void loop()
{


  inData inDataRS, inDataRF;
  lastRecieveRS;
  lastRecieveRF;

  // uint8_t inDataRS[15] = {NULL};
  // uint8_t inDataRF[15] = {NULL};




  if (Serial1.available() >= 14) {

    uint8_t writeNum = 0;
    uint8_t lastRecVal = 0;
    while (lastRecVal != 255 && writeNum < 15){
      lastRecVal = Serial1.read();
      inDataRS.data[writeNum] = lastRecVal;
      writeNum++;

      #ifdef debug
      Serial.write(lastRecVal);
      #endif
    }

    if (checkSumCheck(inDataRS.data) && inDataRS.data[0] == camNum) {
      lastRecieveRS = millis();
      inDataRS.checksum = true;
    }
  }


  if ((lastRecieveRS + ledWaitTime > millis() || lastRecieveRF + ledWaitTime > millis()) && digitalRead(fan1Pin) != HIGH) { 
    digitalWrite(fan1Pin, HIGH);
  }
  else{
    digitalWrite(fan1Pin, LOW);
  }






  if (currentMode.mode == standby || currentMode.mode == moveJoy || currentMode.mode == movePos){

    if(currentMode.modeLast != currentMode.mode && currentMode.modeLast != standby && currentMode.modeLast != moveJoy && currentMode.modeLast != movePos){ // if mode changed
      digitalWrite(enXPin, LOW);
      digitalWrite(enYPin, LOW);
      digitalWrite(fan0Pin, HIGH);
      // digitalWrite(fan1Pin, HIGH);
      currentMode.modeLast = currentMode.mode;
    }


    if (inDataRS.data && inDataRS.checksum){
      handleInData(inDataRS.data);
    }
    else if (inDataRF.data && inDataRF.checksum){
      handleInData(inDataRF.data);
    }


    if (currentMode.mode == moveJoy){

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

  }
  else if (currentMode.mode == halted){

    if(currentMode.modeLast != currentMode.mode){
      xAxis.stop();
      yAxis.stop();
      currentMode.modeLast = currentMode.mode;
    }


  }
  else if (currentMode.mode == error){

    if(currentMode.modeLast != currentMode.mode){
      xAxis.stop();
      yAxis.stop();

      delay(500);

      digitalWrite(enXPin, HIGH);
      digitalWrite(enYPin, HIGH);
      currentMode.modeLast = currentMode.mode;
    }


  }

}





//? Misc

#ifdef debug
void SerialWriteByteArray(uint8_t* data, uint8_t length){
  for (uint8_t i = 0; i < length; i++){
    Serial.write(data[i]);
  }
}
#endif

// void EEPROMCheck(){

//   for (int x = 0; x < EEPROM.length() && x < EEPROM_SIZE; x++){
//     EEPROM.update(x, 0x00);
//   }
  
// }


int16_t stepToDegrees(int16_t step){
  return step * (360 / stepsPerRevolution);
}

int16_t DegreesToStep(int16_t step){
  return step / (360 / stepsPerRevolution);
}


//? RS485 Protocol
uint8_t* receiveCommandRS(){
  if (Serial1.available()) {
  
    uint8_t inData[14] = {NULL};

    uint8_t writeNum = 0;
    uint8_t lastRecVal = 0;
    while (lastRecVal != 255 && writeNum < 15){
      lastRecVal = Serial1.read();
      inData[writeNum] = lastRecVal;
      writeNum++;
    }

    return inData;
  }
  return NULL;
}

uint8_t* receiveCommandRF(){

  return NULL;
}



bool checkSumCheck(uint8_t inData[]){

  // return true;

  uint16_t checkSum = 0;
  for(int x = 0; x < 12; x++){
    if (inData[x] >= 255)
      return false;

    checkSum += inData[x];
  }

  checkSum = checkSum % 255;

  if (checkSum >= 255){
    checkSum = 254;
  }

  if (inData[12] == checkSum){
    return true;
  }
  else{
    return false;
  }

}


void handleInData(uint8_t inData[]){


  currentMode.prevLastCommand = currentMode.lastCommand;
  currentMode.lastCommand = inData[1];
  switch (inData[1])
  {
  case joyUpdateCommand:                  //1
    currentMode.mode = moveJoy;
    joyUpdate(inData[2], inData[3], inData[4], inData[5]);
    break;
  
  case writePosCommand:                   //4
    writePos(inData[2]);
    break;
  
  case callPosCommand:                    //6
    currentMode.mode = movePos;
    callPos(inData[2]);
    break;
  
  case setCoordsCommand:                  //7
    setCoords(inData[2], inData[3], inData[4], inData[5], inData[6], inData[7], inData[8], inData[9]);
    break;
  
  case callErrorCommand:                  //8
    callError(inData[2]);
    break;

  case setZeroCommand:                    //11
    setZero();
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

  vector newPos;
  newPos.x = stepToDegrees(xAxis.currentPosition());
  newPos.y = stepToDegrees(yAxis.currentPosition());
  // newPos.z = stepToDegreestep(yAxis.currentPosition());
  // newPos.a = stepToDegreestep(yAxis.currentPosition());
  if (5 + (posNum-1) * 4 + 4< EEPROM_SIZE)
    EEPROM.put(5 + (posNum-1) * 4, newPos);

}

void callPos(uint8_t posNum){

  xAxis.setMaxSpeed(X_MAX_SPEED);
  xAxis.setAcceleration(X_ACCELERATION);

  yAxis.setMaxSpeed(Y_MAX_SPEED);
  yAxis.setAcceleration(Y_ACCELERATION);

  currentMode.mode = movePos;

  if (posNum == 5){
    currentMode.mode = error;
  }

  vector newPos;
  if (5 + (posNum-1) * 4 + 4< EEPROM_SIZE)
    EEPROM.get(5 + (posNum-1) * 4, newPos);

  xAxis.moveTo(DegreesToStep(newPos.x));
  yAxis.moveTo(DegreesToStep(newPos.y));
  
}

void setCoords(uint8_t xPosA, uint8_t xPosB, uint8_t yPosA, uint8_t yPosB, uint8_t zPosA, uint8_t zPosB, uint8_t aPosA, uint8_t aPosB){

}

void callError(uint8_t errorNum){

}

void setZero(){
  xAxis.setCurrentPosition(0);
  yAxis.setCurrentPosition(0);
}