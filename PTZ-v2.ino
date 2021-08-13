#include "Arduino.h"
#include "TeensyStep.h"

#define fan1Pin PA8

// Define pin connections & motor's steps per revolution
#define enPin PE3
#define stepPin PE5
#define dirPin PE6
//#define enPin PC11
//#define stepPin PA0
//#define dirPin PC15
#define stepsPerRevolution 200*6*4

void setup()
{
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);

  pinMode(fan1Pin, OUTPUT);

  delay(500);

  digitalWrite(enPin, LOW);
  analogWrite(fan1Pin, 255);


  delay(1000);
}
void loop()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor slowly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(2000);
  }
  delay(1000); // Wait a second
  
  // Set motor direction counterclockwise
  digitalWrite(dirPin, LOW);

  // Spin motor quickly
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000);
  }
  delay(1000); // Wait a second
}