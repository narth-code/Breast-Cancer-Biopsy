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

typedef struct{
    u_int16_t x; ///< Steps for X-axis motor.
    u_int16_t y; ///< Steps for Y-axis motor.
    u_int16_t z; ///< Steps for Z-axis motor.
} Positions;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
void bluetoothTask();
void initializeMotors();
void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2);
#endif /* TESTMOTOR_H */
