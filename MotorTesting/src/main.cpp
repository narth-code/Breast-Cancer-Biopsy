
/* Texas A&M University 
** Created on: Feb 22, 2024   
** Author: Harry Tran
** File: main.cpp 
** -------- 
** Allows dynamic control of a stepper motor's speed, acceleration, and movement 
** through serial commands. Supports commands to set speed, acceleration, move to a specific angle,
** and continuously spin or stop the motor. This example is configured for a NEMA17 stepper motor
** with 1/16 microstepping.
**
**    loop() - Main loop handling serial commands to control the stepper motor.
**    -Transforming the motor's rotary motion into linear motion by using a threaded rod:
**    -Threaded rod's pitch = 2 mm. This means that one revolution will move the nut 2 mm.
**    -Default stepping = 400 step/revolution.
**    - 400 step = 1 revolution = 8 mm linear motion. (4 start 2 mm pitch screw)
**    - 1 cm = 10 mm =>> 10/8 * 400 = 4000/8 = 500 steps are needed to move the nut by 1 cm.
 
 
//character for commands

     'H' : Prints all the commands and their functions.
     'R' : Rotates the motor in steps relative to current position.
        Ex: R[(+/-)steps] [steps/second]
     'A' : Rotates the motor to an absolute position.
        Ex: A[(+/-)position] [steps/second]
     'S' : Stops the motor immediately.
     'a' : Sets an acceleration value.
        Ex: a[steps/s^2]
     'L' : Prints the current position/location of the motor.
     'O' : Go to origin, position 0, from current position.
     'U' : Updates the position current position and makes it as the new 0 position.
 */

/* system includes */
#include <Arduino.h>

/* libraries files */
#include <AccelStepper.h>

/* local files */
#include <helperMotor.h>

//User-defined values
long receivedSteps = 0;         //# of steps
long receivedSpeed = 0;         //Steps / second
long receivedAcceleration = 0;  //Steps / second^2
char receivedCommand;
//-------------------------------------------------------------------------------
int direction = 1; // = 1: positive direction, = -1: negative direction
bool newCommand, findLimit, allowRun = false; // booleans for new data from serial, and allowRun flag


// Initialize the stepper library on the pins
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// MARK: SETUP
/**
 * @brief Setup routine to initialize serial communication and stepper motor.
 */
void setup() {
    Serial.begin(BAUD); // Start serial communication at 115200 baud.
    pinMode(LIMIT_SWITCH_1, INPUT);
    pinMode(LIMIT_SWITCH_2, INPUT);
    digitalWrite(LIMIT_SWITCH_1, LOW);  // Turns internal pull-up off

    Serial.println("Motor Testing v1");
    // Set the default values for maximum speed and acceleration:
    Serial.println("Default speed: 1000 steps/s, default acceleration: 400 steps/s^2.");
    stepper.setSpeed(800);
    stepper.setMaxSpeed(800); // steps per second
    stepper.setAcceleration(800); // steps per second squared

    calibrateAxis(LIMIT_SWITCH_1, LIMIT_SWITCH_1);
}

/**
 * loop() - Listens for serial input and executes commands to control the motor's 
 * speed, acceleration, position, and to start or stop spinning. Non-Blocking
 */
void loop() {
    checkSerial(); //check serial port for new commands
    runMotor(); //function to handle the motor  
}
 
void runMotor() {
    if (allowRun == true)
    {
        stepper.enableOutputs(); //enable pins
        if(stepper.distanceToGo() == 0) { // Check if the motor has reached the desired position
            allowRun = false; // Optionally stop running if position is reached
        } else {
           stepper.runSpeedToPosition(); // Run motor to target position at constant speed set by setSpeed();
           //stepper.run(); //step the motor (this will step the motor by 1 step at each loop)
            delayMicroseconds(10);  
        }
    }
    else
    {
        stepper.disableOutputs(); //disable outputs
        return;
    }
}
 
// MARK: SERIAL 
void checkSerial() { //function for receiving the commands
 
    if (Serial.available() > 0) //if there's something in the serial port
    {
        // store the value in the serial port to the receivedCommand variable
        // and enable newCommand flag
        receivedCommand = Serial.read(); 
        newCommand = true; 
        // Only enter this long switch-case statement if there is a new command from the user
        if (newCommand == true) 
        {
            switch (receivedCommand) //we check what is the command
            {
 
            case 'R': //P uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.              
                
                receivedSteps = Serial.parseFloat(); //value for the steps
                //receivedSpeed = Serial.parseFloat(); //value for the speed
                //direction = ((receivedSteps > 0 ? 1 : -1));
                Serial.println((receivedSteps > 0 ? "Positive dir, CCW" : "Negative dir, CW")); //print the action
                moveRelative(); //Run the function
 
                //example: P2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 400 steps/s speed
                //In theory, this movement should take 5 seconds
                break;         
 
            case 'A': //R uses the moveTo() function of the AccelStepper library, which means that it moves absolutely to the current position.            
 
                receivedSteps = Serial.parseFloat(); //value for the steps
                receivedSpeed = Serial.parseFloat(); //value for the speed     
                //direction = ((receivedSteps > 0 ? 1 : -1));
                Serial.println((receivedSteps > 0 ? "Positive dir, CW" : "Negative dir, CCW")); //print the action
                moveAbsolute(); //Run the function
 
                //example: R800 400 - It moves to the position which is located at +800 steps away from 0.
                break;
 
            case 'S': // Stops the motor
               
                stepper.stop(); // stop motor
                stepper.disableOutputs(); //disable power
                Serial.println("Stopped."); // print action
                allowRun = false; // disable running
                break;

            case 's': // Updates the max speed

                allowRun = false; // allowRun disabled, since we are updating a variable
                stepper.disableOutputs(); //disable power
                receivedSpeed = Serial.parseFloat(); //receive the acceleration from serial
                stepper.setAcceleration(receivedSpeed); //update the value of the variable
                Serial.print("New max speed value: "); //confirm update by message
                Serial.print(receivedSpeed); 
                Serial.println(" Steps/second");
                break;
 
            case 'a': // Updates acceleration
 
                allowRun = false; // allowRun disabled, since we are updating a variable
                stepper.disableOutputs(); //disable power
                receivedAcceleration = Serial.parseFloat(); //receive the acceleration from serial
                stepper.setAcceleration(receivedAcceleration); //update the value of the variable
                Serial.print("New acceleration value: "); // confirm update 
                Serial.print(receivedAcceleration); 
                Serial.println("Steps/second^2");
                break;
 
            case 'L': // L: Location
 
                allowRun = false; //we still keep running disabled
                stepper.disableOutputs(); //disable power
                Serial.print("Current location of the motor: ");// Print the message
                Serial.println(stepper.currentPosition()); //Printing the current position in steps.
                break;
               
            case 'O': // O: Origin
 
                allowRun = true;     
                Serial.println("Homing"); // Print the motor status
                goToOrigin();
                break;
 
            case 'U': // Update Origin
 
                allowRun = false; //we still keep running disabled
                stepper.disableOutputs(); //disable power
                stepper.setCurrentPosition(0); // Reset origin to current position             
                Serial.print("The current position is updated to: "); //Print message
                Serial.println(stepper.currentPosition()); // Check position after reset.
                break; 
 
            case 'H':
                //TODO add exapnd help commands
                printCommands(); //Print the commands for controlling the motor
                break;
 
            default:  
                Serial.println("Command Not Found.");
                break;
            }
        }
        // newCommand flag is false so motor doesn't run old commands
        newCommand = false;       
    }
}
 
 
void goToOrigin() {  
    if (stepper.currentPosition() == 0)
    {
        Serial.println("We are at the home position.");
        stepper.disableOutputs(); //disable power
    }
    else
    {
        stepper.setMaxSpeed(400); //set speed manually to 400. In this project 400 is 400 step/sec = 1 rev/sec.
        stepper.moveTo(0); //set abolute distance to move
    }
}
 
void moveRelative() {
    //We move X steps from the current position of the stepper motor in a given direction.
    //The direction is determined by the multiplier (+1 or -1)
   
    allowRun = true; //allow running - this allows entering the RunTheMotor() function.
    //stepper.setMaxSpeed(receivedSpeed); //set speed
    stepper.move(receivedSteps); //set relative distance and direction
}
 
void moveAbsolute() {
    //We move to an absolute position.
    //The AccelStepper library keeps track of the position.
    //The direction is determined by the multiplier (+1 or -1)
    //Why do we need negative numbers? - If you drive a threaded rod and the zero position is in the middle of the rod...
 
    allowRun = true; //allow running - this allows entering the RunTheMotor() function.
    stepper.setMaxSpeed(receivedSpeed); //set speed
    stepper.moveTo(receivedSteps); //set relative distance   
}

// MARK: CALIBRATE AXIS
void calibrateAxis(int limitSwitch1, int limitSwitch2) {
  Serial.println(F("Calibration started."));

  // Approach the first limit switch
  // Move a large distance to ensure it hits the limit
  stepper.moveTo(100000); //Positive is moving towards the motor, rotates Clockwise
  while (digitalRead(limitSwitch1) == LOW) {
    stepper.run();
  }
  Serial.println(F("First Limit Reached"));
  stepper.stop(); // Stop the motor
  stepper.setCurrentPosition(0); // Reset the position to 0
  stepper.move(-200); // Move away from the limit switch

  while (stepper.distanceToGo() != 0) {
    stepper.run(); // Clear the move
  }

  // Approach the second limit switch
  stepper.moveTo(-100000); // Move towards the other end
  while (digitalRead(limitSwitch1) == LOW) {
    stepper.run();
  }
  Serial.println(F("Second Limit Reached"));
  stepper.stop(); // Stop the motor

  Serial.print(F("Max position: "));
  Serial.println(abs(stepper.currentPosition()));

  //stepper.setCurrentPosition(0); // Optionally reset position after calibration
  Serial.println(F("Calibration finished."));
}


void printCommands() {  
    //Printing the commands
    Serial.println(" 'H' : Prints all the commands and their functions.");
    Serial.println(" 'R' : Rotates the motor in steps relative to current position.");
    Serial.println(" 'A' : Rotates the motor to an absolute position.");
    Serial.println(" 'S' : Stops the motor immediately.");
    Serial.println(" 'a' : Sets an acceleration value.");
    Serial.println(" 'L' : Prints the current position/location of the motor.");
    Serial.println(" 'O' : Goes back to 0 position from the current position (homing).");
    Serial.println(" 'U' : Updates the position current position and makes it as the new 0 position. ");   
}