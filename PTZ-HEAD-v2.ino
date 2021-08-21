#include "Arduino.h"

#include "TeensyStep.h"


#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>

#include "commands.h"


#define fan0Pin PC9
#define fan1Pin PA8

// Define pin connections & motor's steps per revolution
#define enXPin PC11
#define enYPin PE3

#define stepsPerRevolutionX 200*6*4
#define stepsPerRevolutionY 200*6*4


// Stepper motorX(PA0, PC15);       // STEP pin: PA0, DIR pin: PC15
// Stepper motorY(PE5, PE6);       // STEP pin: PE5, DIR pin: PE6
Stepper motorX(PA0, PC15), motorY(PE5, PE6);       // STEPx pin: PA0, DIRx pin: PC15 STEPy pin: PE5, DIRy pin: PE6
StepControl stepController;
// RotateControl rotateController;




long long lastRecieve = 0;

void setup()
{

  Serial.begin(115200);

  motorX
    .setAcceleration(1000)
    .setMaxSpeed(15000);
  motorY
    .setAcceleration(500)
    .setMaxSpeed(7500);

  // Declare pins as Outputs
  pinMode(enXPin, OUTPUT);
  pinMode(enYPin, OUTPUT);

  pinMode(fan0Pin, OUTPUT);
  pinMode(fan1Pin, OUTPUT);

  delay(500);

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

  if (Serial.available()) {

    String inData;
    char outData[32];

    inData = Serial.readBytesUntil(0xFF);
    lastRecieve = millis();
  }


  if (lastRecieve + 150 > millis()){
    digitalWrite(fan1Pin, HIGH);
  }
  else{
    digitalWrite(fan1Pin, LOW);
  }


}



//? RS485 Protocol
