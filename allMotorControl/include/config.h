#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
// supported control boards
#define AMS1 (1)
#define AMS2 (2)
#define HG7881 (3) // HG7881 Stepper Driver
#define PASS_STEP (4) // pass-through 4 wire Stepper.h driver
#define CNCV3 (5) // CNC Shield V3 https://blog.protoneer.co.nz/arduino-cnc-shield/

// change this line to select a different control board for your CNC.
#define CONTROLLER CNCV3


#define VERSION        (1)  // firmware version
#define BAUD           (57600)  // How fast is the ESP32 talking?
#define MAX_BUF        (64)  // What is the longest message ESP32 can store?
#define STEPS_PER_TURN (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY (50.0)
#define MAX_FEEDRATE   (1000000.0/MIN_STEP_DELAY)
#define MIN_FEEDRATE   (0.01)
//#define INVERT_Y     (1) // invert Y axis direction

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
extern void m1step(int dir);
extern void m2step(int dir);
extern void disable();
extern void setup_controller();

#endif