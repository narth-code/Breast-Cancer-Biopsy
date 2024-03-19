#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

//#define LIMIT_CALIBRATION (1) //Add if need to calibrate limits

#define VERSION        (1)  // firmware version
#define BAUD           (115200)  // How fast is the ESP32 talking?
#define MAX_BUF        (64)  // What is the longest message ESP32 can store?
#define STEPS_PER_TURN (200)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (50.0)
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.01)
//#define INVERT_Y     (1) // invert Y axis direction

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)


// This is for the Lead 6.35mm(0.25")
// LinearTRAVEL/Revolution(mm) = 6.35
// LinearTRAVEL/step (mm) = 0.03175

#define MM_PER_REV  (6.35) 
#define STEPS_PER_MM ((STEPS_PER_TURN)*16/(MM_PER_REV))


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
extern void m1step(int dir);
extern void m2step(int dir);
extern void disable();
extern void setup_controller();

#endif