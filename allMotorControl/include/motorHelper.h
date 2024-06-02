/* Texas A&M University
** Electronic Systems Engineering Technology
** Author: Harry Tran
** File: testMotor.h
** ----------
** Motors support function
*/

#ifndef MOTORHELPER_H
#define MOTORHELPER_H


/* system includes */
#include <Arduino.h>

/* libraries files */
#include <AccelStepper.h>

/* local helper files */
#include "config.h"

#ifndef VAR_DECLS
# define _DECL extern
# define _INIT(driver, step, dir)
#else
# define _DECL
# define _INIT(driver, step, dir) = x
#endif

// ==============================================================================
// CLASSES
// ==============================================================================
// _DECL AccelStepper stepperX(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
// _DECL AccelStepper stepperY(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
// _DECL AccelStepper stepperZ(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);
typedef struct Node
{
	AccelStepper *stepper = NULL;
    u_short limitSwitch;
	//EasyButton *limitSwitch;
	int maxSteps;
	float speed, acceleration, position;
	
}Motor;

// // ==============================================================================
// // GLOBAL VARIABLES
// // ==============================================================================
// extern long receivedSteps;         //# of steps
// extern long receivedSpeed;         //Steps / second
// extern long receivedAcceleration;  //Steps / second^2
// extern char receivedCommand;
// // ------------------------------------------------------------------------------
// extern bool newCommand, findLimit, allowRun, gotMessage; // booleans for new data from serial, and allowRun flag


// ==============================================================================
// METHODS
// ==============================================================================
extern void checkSerial();
extern void parseData();
extern void initializeMotors();
extern void calibrateAxis(AccelStepper* stepper, int limitSwitch);
#endif /* MOTORHELPER_H */
