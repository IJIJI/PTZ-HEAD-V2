#include "Arduino.h"

#include "TeensyStep.h"

#include <SPI.h>
// #include <nRF24L01.h>
#include <RF24.h>

#include "commands.h"


#define fan1Pin PA8


// Define pin connections & motor's steps per revolution
#define enXPin PC11
#define enYPin PE3

Stepper motorX(PA0, PC15);       // STEP pin: PE5, DIR pin: PE6
// Stepper motorX(PA0, PC15), motorY(PE5, PE6);       // STEP pin: PE5, DIR pin: PE6
StepControl stepController;
// RotateControl rotateController;

// TODO swap motor 1 and 2 pins

#define stepsPerRevolutionX 200*6*4


//! test vars below

long long lastRecieve = 0;

void setup()
{

  Serial.begin(115200);

  motorX
    .setAcceleration(1000)
    .setMaxSpeed(15000);


  // Declare pins as Outputs
  pinMode(enXPin, OUTPUT);
  pinMode(enYPin, OUTPUT);

  pinMode(fan1Pin, OUTPUT);

  delay(500);

  digitalWrite(enXPin, LOW);
  digitalWrite(enYPin, LOW);
  digitalWrite(fan1Pin, HIGH);
  // analogWrite(fan1Pin, 255); //TODO fix analogwrite


  delay(1000);
}
void loop()
{

  if (Serial.available()) {

    String inData;
    char outData[32];

    inData = Serial.readStringUntil('\n');
    lastRecieve = millis();
  }


  if (lastRecieve + 150 > millis()){
    digitalWrite(fan1Pin, HIGH);
  }
  else{
    digitalWrite(fan1Pin, LOW);
  }


}