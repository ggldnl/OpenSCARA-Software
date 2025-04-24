#ifndef CONFIG_HPP
#define CONFIG_HPP

// CNC Shield pinout

const unsigned char STEPPER_1_STEP_PIN = 2; 
const unsigned char STEPPER_1_DIR_PIN = 5;

const unsigned char STEPPER_2_STEP_PIN = 3;
const unsigned char STEPPER_2_DIR_PIN = 6;

const unsigned char STEPPER_3_STEP_PIN = 4;
const unsigned char STEPPER_3_DIR_PIN = 7;

const unsigned char STEPPER_4_STEP_PIN = 12; // Axis A requires an optional jumper
const unsigned char STEPPER_4_DIR_PIN = 13; // Axis A requires an optional jumper

const unsigned char ENABLE = 8;  // Active-low (i.e. LOW turns on the drivers)

const unsigned char STEPPER_1_ENDSTOP_PIN = A3; // Should be pin 17 on Arduino Uno
const unsigned char STEPPER_2_ENDSTOP_PIN = 9;
const unsigned char STEPPER_3_ENDSTOP_PIN = 10;
const unsigned char STEPPER_4_ENDSTOP_PIN = 11;

// Velocity and acceleration

/*
 * I empirically found that with my combination of steppers and drivers
 * the stepper work best if we use a pulse with a frequency between
 * 400 and 800 microseconds. A shorter frequency results in a higher
 * velocity. The stepper class works with velocity and accelerations 
 * expressed in steps/s and steps/s^2 respectively. This means that 
 * we would have to use a minimum and maximum velocity in the range 
 * of 1250 to 2500 steps/s, according to the following computation:
 *
 * delay = 1000000/velocity
 * 400 = 1000000/x -> x = 2500
 * 800 = 1000000/x -> x = 1250
 * 
 * The above holds for microstepping 1. Switching to microstepping 16
 * allowed for a wider range of velocities and accelerations.
 */

const double MAX_VELOCITY_STEPS_S = 10000.0;
const double MIN_VELOCITY_STEPS_S = 500.0;
const double ACCELERATION = 5000.0;

// Homing

const double HOMING_VELOCITY_STEPS_S = 1000.0;

/*
 * The maximum number of steps to be used for homing. During homing,
 * the stepper will move at a constant speed until it reaches the
 * limit switch. The maximum number of steps is used to prevent the
 * stepper from running indefinitely in case the limit switch is
 * broken or not connected.
 */
const long MAX_HOMING_STEPS = 1000000;

// Reduction ratios

const int STEPPER_1_MICROSTEPPING = 16;  // You can change these with jumpers on the CNC shield
const int STEPPER_2_MICROSTEPPING = 16;
const int STEPPER_3_MICROSTEPPING = 16;
const int STEPPER_4_MICROSTEPPING = 16;

const float STEPPER_1_STEPS_PER_REVOLUTION = 200.0;  // These depend on the stepper motor
const float STEPPER_2_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_3_STEPS_PER_REVOLUTION = 200.0;
const float STEPPER_4_STEPS_PER_REVOLUTION = 200.0;

/*
 * To compute the number of steps per meter, we need to divide the
 * number of steps per revolution by the lead of the screw. The lead
 * is the distance the screw moves in one revolution.
 */
const float JOINT_1_LEAD = 8.0;  // Leadscrew pitch, mm
const float JOINT_1_REDUCTION = JOINT_1_LEAD / 1000.0;

/*
 * To compute the reduction ratio in a belt-driven pulley system, 
 * you divide the number of teeth on the driven pulley (the larger one) 
 * by the number of teeth on the driver pulley (the one connected to the motor).
 * In our case, all the pulleys on the motor shaft are 16 teeth, and the
 * driven pulleys all have 64 teeth. The reduction ratio is always 64/16 = 4.
 */
const float JOINT_2_REDUCTION = 4;  // Reduction on the first revolute joint
const float JOINT_3_REDUCTION = 4;  // Reduction on the second revolute joint
const float JOINT_4_REDUCTION = 4;  // Reduction on the third revolute joint

/*
 * Once we have these constants, we can compute the number of steps per
 * meter for the first joint and the number of steps per radian for the 
 * other joints simply by multiplying the target quantity by the respective
 * joint constant.
 * e.g. we want to move the first joint (prismatic) by 0.1 m: 0.1 m * JOINT_1_STEPS_PER_M
 * e.g. we want to move the third joint (revolute) by 0.1 rad: 0.1 rad * JOINT_3_STEPS_PER_RAD
 */
const double JOINT_1_STEPS_PER_M = (STEPPER_1_STEPS_PER_REVOLUTION * STEPPER_1_MICROSTEPPING) / JOINT_1_REDUCTION;                 // steps/m
const double JOINT_2_STEPS_PER_RAD = (STEPPER_2_STEPS_PER_REVOLUTION * STEPPER_2_MICROSTEPPING / (2 * M_PI)) * JOINT_2_REDUCTION;  // steps/rad
const double JOINT_3_STEPS_PER_RAD = (STEPPER_3_STEPS_PER_REVOLUTION * STEPPER_3_MICROSTEPPING / (2 * M_PI)) * JOINT_3_REDUCTION;  // steps/rad
const double JOINT_4_STEPS_PER_RAD = (STEPPER_3_STEPS_PER_REVOLUTION * STEPPER_4_MICROSTEPPING / (2 * M_PI)) * JOINT_4_REDUCTION;  // steps/rad

// Link lenghts

// First link is prismatic
const float LINK_2_LENGTH = 0.120;  // Link 2 length in mm
const float LINK_3_LENGTH = 0.120;  // Link 3 length in mm

// Joint limits

// The first joint angle can move within the [0, 200] mm range
const float JOINT_1_MIN = 0;
const float JOINT_1_MAX = 0.2;  // 200 mm, 0.2 m

// The second joint angle can move within the [-pi/2, +pi/2] range
const float JOINT_2_MIN = -M_PI / 2;
const float JOINT_2_MAX = M_PI / 2;

// The third joint angle can move by almost 360 degrees
const float JOINT_3_MIN = 0;
const float JOINT_3_MAX = 5.93412;  // 340 degrees

// The fourth joint angle can also move by almost 360 degrees
const float JOINT_4_MIN = 0;
const float JOINT_4_MAX = 5.93412;  // 340 degrees

#endif // CONFIG_HPP
