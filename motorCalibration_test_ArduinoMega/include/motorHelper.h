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
#define stepsPerRevolution 400  

// Define the STEP and DIRECTION pin connections
#define stepPin 2
#define dirPin 5


#define LIMIT_SWITCH_1 //6
#define LIMIT_SWITCH_2 //7
// put function declarations here:
extern void calibrateAxis(AccelStepper& stepper, int limitSwitch1, int limitSwitch2);
extern void establishOrigin(AccelStepper& stepper, int limitSwitch1);
#endif /* HELPERMOTOR_H */
