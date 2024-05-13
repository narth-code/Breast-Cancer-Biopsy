#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define LIMIT_CALIBRATION (1) //Add if need to calibrate limits

#define VERSION        (2)      // firmware version
#define BAUD           (115200) // How fast is the ESP32 talking?
#define MAX_BUF        (64)     // What is the longest message ESP32 can store?
#define STEPS_PER_TURN (200)    // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (50.0)
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.01)
#define MOTOR_SPEED    (4500)   // Step/s
#define MICROSTEP      (4)
#define MAX_STEPS      (44920)
//#define INVERT_Y     (1) // invert Y axis direction

// This is for the Lead 6.35mm(0.25")
// LinearTRAVEL/Revolution(mm) = 6.35
// LinearTRAVEL/step (mm) = 0.03175

#define MM_PER_REV  (2) 
#define STEPS_PER_MM ((STEPS_PER_TURN)*MICROSTEP/(2*MM_PER_REV))

//#define LIMIT_CALIBRATION
/**
 * @def MOTOR_{axis}_STEP_PIN, MOTOR_{axis}_DIR_PIN
 * Configure the GPIO pin number for the stepper motor 
 * step and direction signal.
 */

/*
36(I)-                      23(X)-
39(I)-                      22   -
34(I)-                      1(X) -
35(I)-                      3(X) - 
32   -                      21   -MOTOR_X_DIR_PIN
33(X)-                      19   -MOTOR_Y_DIR_PIN
25(X)-                      18   -MOTOR_Z_DIR_PIN
26   -                      5*   -
27   -MOTOR_Z_STEP_PIN      17   -MOTOR_X_STEP_PIN
14   -                      16   -MOTOR_Y_STEP_PIN
12*  -                      4(X) -
                            0(X) -
13(X)-                      2    -
                            15*  -  
*/
/* Motor 1 */
#define MOTOR_X_STEP_PIN 18
#define MOTOR_X_DIR_PIN 19
/* Motor 2 */
#define MOTOR_Y_STEP_PIN 17
#define MOTOR_Y_DIR_PIN 21
/* Motor 3 */
#define MOTOR_Z_STEP_PIN 27
#define MOTOR_Z_DIR_PIN 18

/* LIMIT SWITCHES */
#define LIMIT_SWITCH_X 26 
#define LIMIT_SWITCH_Y 25 
#define LIMIT_SWITCH_Z 14 


#endif