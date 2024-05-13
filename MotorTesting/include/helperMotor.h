/* Texas A&M University
** Created on: Feb 23, 2024
** Author: Harry Tran
** File: helperMotor.h
** ----------
** Motors support function
*/

#ifndef HELPERMOTOR_H
#define HELPERMOTOR_H

#define BAUD 115200
#define STEPS_PER_TURN      (200)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY_US   (50.0)
#define MICROSTEP           (4)
/* Definitions for calculating steps per revolution based on stepper characteristics
 *      stepMode: 1/2 = 2, 1/4 = 4, 1/8 = 8, 1/16 = 16
 *      stepAngle: degrees/step
 */
//#define INVERT_Y     (1) // invert Y axis direction

// This is for the Lead 6.35mm(0.25")
// LinearTRAVEL/Revolution(mm) = 6.35
// LinearTRAVEL/step (mm) = 0.03175

#define MM_PER_REV  (2) 
#define STEPS_PER_MM ((STEPS_PER_TURN)*MICROSTEP/2*(MM_PER_REV))


// Define the STEP and DIRECTION pin connections
#define LIMIT_SWITCH_X 26 
#define LIMIT_SWITCH_Y 25
#define stepPin_Y 17 
#define dirPin_Y 21 
#define stepPin_X 18 
#define dirPin_X 19 

// put function declarations here:
void runMotor();
void checkSerial();
void goToOrigin();
void moveRelative();
void moveAbsolute();
void calibrateAxis(AccelStepper* stepper, int limitSwitch1, int limitSwitch2); 
void printCommands();

#endif /* HELPERMOTOR_H */
