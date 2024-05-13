/* Texas A&M University
** Electronic Systems Engineering Technology
** Author: Harry Tran
** File: testMotor.h
** ----------
** Motors support function
*/

#ifndef TESTMOTOR_H
#define TESTMOTOR_H

#include <stdint.h>


/* Global variable */
bool gotMessage = false; //< Flag to indicate message reception.
enum {
    X,
    Y,
    Z
};



//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
void serialTask();
void initializeMotors();
void initializeButtons();
void parseData(String data);
void setSteppers();
void calibrateAxis(AccelStepper* stepper, EasyButton* limitSwitch);

void buttonISR();
#endif /* TESTMOTOR_H */
