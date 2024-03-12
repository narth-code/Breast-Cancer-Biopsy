/* Texas A&M University
** Created on: Mar 5, 2024
** Author: Harry Tran
** File: helperMotor.h
** ----------
** Motors support function
*/

#ifndef HELPERMOTOR_H
#define HELPERMOTOR_H
/*
36(I)-              23(X)-
39(I)-              22   -
34(I)-              1(X) -
35(I)-              3(X) - 
32   -              21   -dirPin_1
33(X -              19   -
25(X)-              18   -
26   -              5*   -
27   -              17   -stepPin_1
14   -              16   -
12*  -              4(X) -
                    0(X) -
13(X)-              2    -
                    15*  -  
*/
#include <AccelStepper.h>
#include <Arduino.h>
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
extern void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2);

#endif /* HELPERMOTOR_H */
