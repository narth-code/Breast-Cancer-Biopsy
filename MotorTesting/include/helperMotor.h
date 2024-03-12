/* Texas A&M University
** Created on: Feb 23, 2024
** Author: Harry Tran
** File: helperMotor.h
** ----------
** Motors support function
*/

#ifndef HELPERMOTOR_H
#define HELPERMOTOR_H


/* Definitions for calculating steps per revolution based on stepper characteristics
 *      stepMode: 1/2 = 2, 1/4 = 4, 1/8 = 8, 1/16 = 16
 *      stepAngle: degrees/step
 */
#define stepsPerRevolution(stepAngle, stepMode) ((int)stepMode * (int)360/(double)stepAngle)  
#define NEMA17 stepsPerRevolution(1.8, 16) // NEMA17 with 1.8 degree steps and 1/16 microstepping
#define NEMA14 stepsPerRevolution(1.8, 1)  // Example for NEMA14 with FULL step

// Define the STEP and DIRECTION pin connections
#define stepPin 17
#define dirPin 21

// put function declarations here:

#endif /* HELPERMOTOR_H */
