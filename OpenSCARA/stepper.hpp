#ifndef STEPPER_MOTOR_HPP
#define STEPPER_MOTOR_HPP


class SpeedProfile {
  
  public:
  
    virtual void compute(long totalSteps, double initialVelocity, double finalVelocity = 0, double maxVelocity = 0, double acceleration = 0) = 0;
    virtual double update(long currentStep) = 0;
};

class TrapezoidalSpeedProfile : public SpeedProfile {
  /**
   * Accelerates from an initial velocity to a maximum velocity over a set number of steps,
   * maintains the maximum velocity for a whyle and then decelerates to a final velocity.
   * If the total distance is too short for full acceleration and deceleration phases, 
   * it falls back to a triangular profile (no constant-speed phase).
   */

  private:
  
    double maxVelocity;
    double initialVelocity, finalVelocity, acceleration;
    long accelSteps, decelSteps, constSteps, totalSteps;

  public:
    
    void compute(long _totalSteps, double _initialVelocity, double _finalVelocity = 0, double _maxVelocity = 0, double _acceleration = 0) override {
      totalSteps = _totalSteps;
      initialVelocity = _initialVelocity;
      maxVelocity = _maxVelocity;
      finalVelocity = _finalVelocity;
      acceleration = _acceleration;

      accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
      decelSteps = abs((maxVelocity * maxVelocity - finalVelocity * finalVelocity) / (2 * acceleration));
      constSteps = totalSteps - (accelSteps + decelSteps);

      // Fallback to triangular profile if necessary
      if (accelSteps + decelSteps > totalSteps) {
          maxVelocity = sqrt(acceleration * totalSteps + 0.5 * (initialVelocity * initialVelocity + finalVelocity * finalVelocity));
          accelSteps = abs((maxVelocity * maxVelocity - initialVelocity * initialVelocity) / (2 * acceleration));
          decelSteps = totalSteps - accelSteps;
          constSteps = 0;
      }
    }

    double update(long currentStep) override {
      if (currentStep < accelSteps) {
        return sqrt(initialVelocity * initialVelocity + 2 * acceleration * currentStep);
      } else if (currentStep < totalSteps - decelSteps) {
        return maxVelocity;
      } else {
        return sqrt(finalVelocity * finalVelocity + 2 * acceleration * (totalSteps - currentStep));
      }
    }
};

class LinearSpeedProfile : public SpeedProfile {
  /**
   * Linearly interpolates the velocity from an initial velocity to a final velocity over a set number of steps.
   */

  private:
    
    double initialVelocity, finalVelocity, increment;
    long totalSteps;

  public:
    
    void compute(long _totalSteps, double _initialVelocity, double _finalVelocity = 0, double _maxVelocity = 0, double _acceleration = 0) override {
      totalSteps = _totalSteps;
      initialVelocity = _initialVelocity;
      finalVelocity = _finalVelocity;

      // Fixed increment
      increment = (finalVelocity - initialVelocity) / totalSteps;
    }

    double update(long currentStep) override {
      // Linearly interpolate the velocity
      return initialVelocity + currentStep * increment;
    }
};

class ConstantSpeedProfile : public SpeedProfile {
  /**
   * Keeps the velocity constant throughout the entire movement.
   * This is useful for simple applications where acceleration and deceleration are not needed.
   */
  
  private:
    
    double velocity;
    long totalSteps;

  public:
    
    void compute(long _totalSteps, double _initialVelocity, double _finalVelocity = 0, double _maxVelocity = 0, double _acceleration = 0) override {
      totalSteps = _totalSteps;
      velocity = _initialVelocity;
    }

    double update(long currentStep) override {
      // Return velocity unchanged
      return velocity;
    }
};

class StepperMotor {
  /**
   * A class to control a stepper motor using a speed profile.
   * The class allows for different speed profiles: constant, linear, and trapezoidal.
   * It also provides methods to move to a target position and check if the target is reached.
   */
  
  private:
    
    // Pins
    uint8_t pulPin, dirPin;
    
    // Position
    long currentPosition, targetPosition;
    long totalSteps, currentStep;
    bool direction;
    
    // Velocity
    unsigned long stepInterval, lastStepTime;
    double currentVelocity;
    
    // Speed profile
    SpeedProfile* speedProfile; // Pointer to current speed profile
    TrapezoidalSpeedProfile trapezoidalProfile;
    LinearSpeedProfile linearProfile;
    ConstantSpeedProfile constantProfile;
    
    void initializeMove(long _targetPosition, double _initialVelocity) {
        targetPosition = _targetPosition;
        currentStep = 0;
        
        totalSteps = abs(targetPosition - currentPosition);
        direction = (targetPosition > currentPosition) ? HIGH : LOW;
        digitalWrite(dirPin, direction);
        
        // Set initial delay
        stepInterval = 1e6 / _initialVelocity;
        lastStepTime = micros();
    }

  public:
    
    StepperMotor(uint8_t _pulPin, uint8_t _dirPin) : 
        pulPin(_pulPin), dirPin(_dirPin),
        currentPosition(0), targetPosition(0),
        currentVelocity(0),
        totalSteps(0), currentStep(0),
        stepInterval(0), lastStepTime(0), 
        speedProfile(nullptr),
        direction(true) 
    {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
    }

    bool isAtTarget() {
      /**
       * Check if the stepper has reached the target position.
       */
        return currentPosition == targetPosition;
    }

    void moveToPosition(long _targetPosition, double _initialVelocity) {
      speedProfile = &constantProfile;
      initializeMove(_targetPosition, _initialVelocity);
      speedProfile->compute(totalSteps, _initialVelocity);
    }
    
    void moveToPosition(long _targetPosition, double _initialVelocity, double _finalVelocity) {
      /**
       * Move to a target position with a linear speed profile.
       */
      speedProfile = &linearProfile;
      initializeMove(_targetPosition, _initialVelocity);
      speedProfile->compute(totalSteps, _initialVelocity, _finalVelocity);
    }
    
    void moveToPosition(long _targetPosition, double _initialVelocity, double _maxVelocity, double _finalVelocity, double _acceleration) {
      /**
       * Move to a target position with a trapezoidal speed profile.
       */
      speedProfile = &trapezoidalProfile;
      initializeMove(_targetPosition, _initialVelocity);
      speedProfile->compute(totalSteps, _initialVelocity, _finalVelocity, _maxVelocity, _acceleration);
    }

    void step() {
        
        unsigned long currentTime = micros();
        unsigned long elapsedTime = currentTime - lastStepTime;

        if (currentPosition != targetPosition && elapsedTime >= stepInterval) {

            currentPosition += (direction == HIGH) ? 1 : -1;
            currentStep ++;
            
            if (speedProfile) {
                currentVelocity = speedProfile->update(currentStep);
                stepInterval = 1e6 / currentVelocity;
            }

            lastStepTime = currentTime;

            digitalWrite(pulPin, HIGH);
            delayMicroseconds(1);
            digitalWrite(pulPin, LOW);
        }
    }

    void setCurrentPosition(long _currentPosition) {
        currentPosition = _currentPosition;
    }

    long getCurrentPosition() {
        return currentPosition;
    }

    long getTargetPosition() {
        return targetPosition;
    }

    double getCurrentVelocity() {
        return currentVelocity;
    }
};

#endif // STEPPER_MOTOR_HPP