#include "config.hpp"
#include "logger.hpp"
#include "button.hpp"
#include "vector.hpp"
#include "stepper.hpp"
#include "kinematics.hpp"

// Function declarations
void moveLine(const TaskPoint&, const TaskPoint&, const double = MIN_VELOCITY_STEPS_S, const double = MIN_VELOCITY_STEPS_S, const double =  MIN_VELOCITY_STEPS_S, const int = 20);
void homeAxis(StepperMotor&, Button&, long, double = HOMING_VELOCITY_STEPS_S);
void homeAll();
void moveAll();
void disable();
void enable();

// Current robot configuration 
ConfigurationPoint currentConfig; // (q1, q2, q3, q4)
TaskPoint currentEndEffector; // (x, y, z, yaw)

// Define the steppers
StepperMotor stepper1(STEPPER_1_STEP_PIN, STEPPER_1_DIR_PIN);
StepperMotor stepper2(STEPPER_2_STEP_PIN, STEPPER_2_DIR_PIN);
StepperMotor stepper3(STEPPER_3_STEP_PIN, STEPPER_3_DIR_PIN);
StepperMotor stepper4(STEPPER_4_STEP_PIN, STEPPER_4_DIR_PIN);

// Define the buttons
Button endstop1(STEPPER_1_ENDSTOP_PIN);
Button endstop2(STEPPER_2_ENDSTOP_PIN);
Button endstop3(STEPPER_3_ENDSTOP_PIN);
Button endstop4(STEPPER_4_ENDSTOP_PIN);

void setup() {

  delay(2000);

  // Initialize serial communication
  Serial.begin(115200);

  // Set log level to INFO
  Logger::setLogLevel(Logger::DEBUG);

  // Enable the board
  pinMode(ENABLE, OUTPUT);
  enable();

  // Home axis
  homeAll();
  Logger::info("All axis homed.");

  TaskPoint target(0.1, 0.1, 0.1, 3.14);
  Logger::info("Moving straight ({}, {}, {}, {}) -> ({}, {}, {}, {})", currentEndEffector.x, currentEndEffector.y, currentEndEffector.z, currentEndEffector.yaw, target.x, target.y, target.z, target.yaw);
  moveLine(currentEndEffector, target);

  // Introduce a small delay (2s)
  delay(2000);
}

void moveLine(
  const TaskPoint& start,
  const TaskPoint& end,
  const double initialVelocity, 
  const double maxVelocity,
  const double finalVelocity,
  const int numPoints
) {
  /**
   * Moves from the start point to the end point (both in Task space) in a 
   * straight line possibly with a trapezoidal speed profile (start with 
   * initialVelocity, cruise at maxVelocity and decelerate to finalVelocity). 
   * numPoints tells how many intermediate points we should use to compute 
   * the inverse kinematics solutions.
   */

  // Precompute the trajectory in configuration space
  Vector<TaskPoint> interpolatedPoints = interpolateLine3D(start, end, numPoints);
  Vector<ConfigurationPoint> trajectory;
  for (int i = 0; i < interpolatedPoints.getSize(); ++i) {

    // Take the target point
    TaskPoint t = interpolatedPoints[i];

    // Compute inverse kinematics solutions
    ConfigurationPoint c1, c2;
    bool result = inverseKinematics(t, c1, c2);

    // A point in the trajectory is not reachable
    if (!result) {
      Logger::error("Point ({}, {}, {}) is not reachable.", t.x, t.y, t.z);
      return;
    }

    // TODO select configuration
    trajectory.pushBack(c1);
  }

  // Now we have a trajectory
  for (int i = 0; i < trajectory.getSize(); ++i) {
    
    ConfigurationPoint nextConfig = trajectory[i];

    // Compute the steps it takes to reach the configuration
    long q1Steps = nextConfig.q1 * JOINT_1_STEPS_PER_M;
    long q2Steps = nextConfig.q2 * JOINT_2_STEPS_PER_RAD;
    long q3Steps = nextConfig.q3 * JOINT_3_STEPS_PER_RAD;
    long q4Steps = nextConfig.q4 * JOINT_4_STEPS_PER_RAD;

    // Find longest distance to cover
    long distance1 = abs(stepper1.getCurrentPosition() - q1Steps);
    long distance2 = abs(stepper2.getCurrentPosition() - q2Steps);
    long distance3 = abs(stepper3.getCurrentPosition() - q3Steps);
    long distance4 = abs(stepper4.getCurrentPosition() - q4Steps);

    double maxSteps = abs(max(distance1, max(distance2, max(distance3, distance4))) * 1.0);

    /*
     * The motor that needs to move the most steps should move at full velocity and acceleration
     * while the other motor's velocity and acceleration are scaled proportionally to match 
     * the duration of the motion of the first one. We use 1 if the result is zero to prevent
     * divition by 0
     */
    double scaleFactor1 = (distance1 != 0) ? (maxSteps / distance1) : 1.0;
    double scaleFactor2 = (distance2 != 0) ? (maxSteps / distance2) : 1.0;
    double scaleFactor3 = (distance3 != 0) ? (maxSteps / distance3) : 1.0;
    double scaleFactor4 = (distance4 != 0) ? (maxSteps / distance4) : 1.0;

    // Define velocities
    double v1i = i == 0 ? initialVelocity : maxVelocity;
    double v2i = i == 0 ? initialVelocity : maxVelocity;
    double v3i = i == 0 ? initialVelocity : maxVelocity;
    double v4i = i == 0 ? initialVelocity : maxVelocity;

    double v1f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;
    double v2f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;
    double v3f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;
    double v4f = i == trajectory.getSize() - 1 ? finalVelocity : maxVelocity;

    // Scale velocities and accelerations
    v1i = v1i / scaleFactor1;
    v2i = v2i / scaleFactor2;
    v3i = v3i / scaleFactor3;
    v4i = v4i / scaleFactor4;

    v1f = v1f / scaleFactor1;
    v2f = v2f / scaleFactor2;
    v3f = v3f / scaleFactor3;
    v4f = v4f / scaleFactor4;

    stepper1.moveToPosition(q1Steps, v1i, v1f);
    stepper2.moveToPosition(q2Steps, v2i, v2f);
    stepper3.moveToPosition(q3Steps, v3i, v3f);
    stepper4.moveToPosition(q4Steps, v4i, v4f);
 
    moveAll();

    // Update current configuration
    currentConfig = nextConfig;
  }
}

void moveAll() {
  /**
   * Move all steppers until they reach the target position.
   */

  while (!(
    stepper1.isAtTarget() && 
    stepper2.isAtTarget() && 
    stepper3.isAtTarget() && 
    stepper4.isAtTarget()
  )) {

    /* If one of the motors terminates the movement, it simply stops
     * while the others keep moving
     */

    stepper1.step();
    stepper2.step();
    stepper3.step();
    stepper4.step();
  }
}

/* --------------------------------- Homing --------------------------------- */

void homeAxis(
    StepperMotor& stepper, 
    Button& limitSwitch, 
    long homingSteps, 
    double velocity
  ) {
  /**
   * Home the specified axis by moving the motor with constant speed until the button fires.
   * The motor stops after #homingSteps anyway. We can't use a tapezoidal speed profile
   * since we don't know in advance how many steps we need to take and thus we cannot
   * decelerate properly. We could gradually accelerate until we reach the maximum
   * velocity but I prefer to cruise to the limit switch with minimum velocity 
   * in order to move safely to the target.
   */

  stepper.moveToPosition(homingSteps, velocity);
  while (!stepper.isAtTarget()) {

    if (limitSwitch.pressed()) {
      Logger::debug("Endstop {} reached.", limitSwitch.getPin());
      break;
    }

    // Perform a step with constant velocity
    stepper.step();

    // Little delay to smooth things out
    // delayMicroseconds(10);  // TODO check if needed
  }

  // Set the zero
  stepper.setCurrentPosition(0);
}

void homeAll() {
  /**
   * Home all the axis.
   */

  // Move the first axis down for 10000 steps or until the limit switch registers a press
  Logger::debug("Homing axis 1...");
  homeAxis(stepper1, endstop1, -MAX_HOMING_STEPS);
  Logger::debug("Axis 1 homed.");

  // Start from the last revolute joint and go backwards homing the others
  Logger::debug("Homing axis 4...");
  homeAxis(stepper4, endstop4, MAX_HOMING_STEPS);
  Logger::debug("Axis 4 homed.");

  Logger::debug("Homing axis 3...");
  homeAxis(stepper3, endstop3, -MAX_HOMING_STEPS);
  Logger::debug("Axis 3 homed.");

  Logger::debug("Homing axis 2...");
  homeAxis(stepper2, endstop2, MAX_HOMING_STEPS);
  Logger::debug("Axis 2 homed.");

  // TODO check this
  // Move a little upwards to leave clearance for the endstop
  float q1_displacement = 0.01;  // 1cm 
  stepper1.moveToPosition(q1_displacement * JOINT_1_STEPS_PER_M, MAX_HOMING_STEPS);
  currentConfig.q1 = q1_displacement;  // Update current configuration

  // Move the second axis such that it points forward
  float q2_displacement = 1.5708;
  stepper2.moveToPosition(q2_displacement * JOINT_2_STEPS_PER_RAD, MAX_HOMING_STEPS);
  currentConfig.q2 = q2_displacement;
}

void enable() {
  digitalWrite(ENABLE, LOW);
  Logger::debug("Steppers enabled.");
}

void disable() {
  digitalWrite(ENABLE, HIGH);
  Logger::debug("Steppers disabled.");
}

/* ---------------------------------- Loop ---------------------------------- */

void loop() {}
