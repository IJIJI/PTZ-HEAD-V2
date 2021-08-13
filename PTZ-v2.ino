#include "Arduino.h"
#include "TeensyStep.h"




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

void setup()
{

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

  motorX.setTargetAbs(1000);  // Set target position to 1000 steps from current position


  stepController.move(motorX);    // Do the move

  delay(1000); // Wait a second
  
  motorX.setTargetAbs(-1000);  // Set target position to 1000 steps from current position
  stepController.move(motorX);    // Do the move

  delay(1000); // Wait a second
}