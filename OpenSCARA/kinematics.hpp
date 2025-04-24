#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "config.hpp"
#include "vector.hpp"
#include <math.h>

// #include "logger.hpp"

#ifdef min
  #undef min
#endif

#ifdef max
  #undef max
#endif

template<typename T>
inline T min(T a, T b) {
  return (a < b) ? a : b;
}

template<typename T>
inline T max(T a, T b) {
  return (a > b) ? a : b;
}

template<typename T>
inline T clamp(T val, T min_val, T max_val) {
  return max(min(val, max_val), min_val);
}

struct TaskPoint {
  /**
   * Represent a point in Task space. The Task space is the space in which the end-effector 
   * of the robot operates. It represents the positions and orientations of the end-effector 
   * in the workspace. In our case we will have position + yaw.
   */
    
  float x, y, z, yaw;

  TaskPoint() : x(0), y(0), z(0), yaw(0) {}
  TaskPoint(float _x, float _y, float _z) : x(_x), y(_y), z(_z), yaw(0) {}
  TaskPoint(float _x, float _y, float _z, float _yaw) : x(_x), y(_y), z(_z), yaw(_yaw) {}
};


struct ConfigurationPoint {
  /**
   * Represent a point in Configuration space. The Configuration space is a mathematical 
   * representation of all possible configurations of the system. Each point in this space 
   * corresponds to a particular robot configuration (a set of joint values). In our case
   * we have 4 joints (4 DoF).
   */

  float q1, q2, q3, q4;  // Joint angles

  // The compiler should automatically generate an assignment operator
  ConfigurationPoint() : q1(0), q2(0), q3(0), q4(0) {}
  ConfigurationPoint(float _q1, float _q2, float _q3, float _q4) : q1(_q1), q2(_q2), q3(_q3), q4(_q4) {}
};

bool inverseKinematics(const TaskPoint& t, ConfigurationPoint& c1, ConfigurationPoint& c2) {

  // Check prismatic joint (joint 1) limits  
  if (!(JOINT_1_MIN <= t.z && t.z <= JOINT_1_MAX)) {
    // Logger::debug("Point z ({}) is out of prismatic joint range [{}, {}].", t.z, JOINT_1_MIN, JOINT_1_MAX);
    return false;
  }

  if (!(JOINT_4_MIN <= t.yaw && t.yaw <= JOINT_4_MAX)) {
    // Logger::debug("Point yaw ({}) is out of revolute joint range [{}, {}].", t.yaw, JOINT_4_MIN, JOINT_4_MAX);
    return false;
  }

  // Compute the planar distance from the origin to the target (in the x-y plane)
  float distance = sqrt(t.x * t.x + t.y * t.y);
  
  if (distance > LINK_2_LENGTH + LINK_3_LENGTH || distance < fabs(LINK_2_LENGTH - LINK_3_LENGTH)) {
    // Logger::debug("Point ({}, {}) is unreachable by the planar mechanism.", t.x, t.y);
    return false;
  }

  /*
  float cos_q3 = (distance * distance - LINK_2_LENGTH*LINK_2_LENGTH - LINK_3_LENGTH*LINK_3_LENGTH) / (2 * LINK_2_LENGTH * LINK_3_LENGTH);

  // Clamp due to possible numerical issues
  cos_q3 = clamp<float>(cos_q3, -1.0, 1.0);

  // Compute the two possible solutions for q3 (elbow up and down)
  float q3_a = acos(cos_q3);     // Candidate A
  float q3_b = -q3_a;            // Candidate B

  // Compute the angle theta from the origin to the target point
  float theta = atan2(t.y, t.x);

  // Candidate A:
  float sin_q3_a = sin(q3_a);
  float phi_a = atan2(LINK_3_LENGTH * sin_q3_a, LINK_2_LENGTH + LINK_3_LENGTH * cos(q3_a));
  float q2_a = theta - phi_a;
  
  // Candidate B:
  float sin_q3_b = sin(q3_b);
  float phi_b = atan2(LINK_3_LENGTH * sin_q3_b, LINK_2_LENGTH + LINK_3_LENGTH * cos(q3_b));
  float q2_b = theta - phi_b;

  // The prismatic joint is taken directly from the target z. Same for both solutions
  float q1 = t.z;

  // Compute wrist angle
  float q4_a = t.yaw - (q2_a + q3_a);
  float q4_b = t.yaw - (q2_b + q3_b);
  */

  // The prismatic joint is taken directly from the target z. Same for both solutions
  float q1 = t.z;

  float c3 = (t.x * t.x + t.y * t.y - LINK_2_LENGTH * LINK_2_LENGTH - LINK_3_LENGTH * LINK_3_LENGTH) / (2 * LINK_2_LENGTH * LINK_3_LENGTH);
  float s3_a = sqrt(1 - c3 * c3);
  float s3_b = -s3_a;

  float q3_a = atan2(s3_a, c3);
  float q3_b = atan2(s3_b, c3);

  float gamma = atan2(t.y, t.x);
  float q2_a = gamma - atan2(LINK_3_LENGTH * s3_a, LINK_2_LENGTH + LINK_3_LENGTH * c3);
  float q2_b = gamma - atan2(LINK_3_LENGTH * s3_b, LINK_2_LENGTH + LINK_3_LENGTH * c3);

  float q4_a = t.yaw - (q2_a + q3_a);
  float q4_b = t.yaw - (q2_b + q3_b);

  // Verify that each candidate solution satisfies the joint limits
  bool validA =  (q1 >= JOINT_1_MIN && q1 <= JOINT_1_MAX) &&
                 (q2_a >= JOINT_2_MIN && q2_a <= JOINT_2_MAX) &&
                 (q3_a >= JOINT_3_MIN && q3_a <= JOINT_3_MAX) &&
                 (q4_a >= JOINT_4_MIN && q4_a <= JOINT_4_MAX);
  
  bool validB =  (q1 >= JOINT_1_MIN && q1 <= JOINT_1_MAX) &&
                 (q2_b >= JOINT_2_MIN && q2_b <= JOINT_2_MAX) &&
                 (q3_b >= JOINT_3_MIN && q3_b <= JOINT_3_MAX) &&
                 (q4_b >= JOINT_4_MIN && q4_b <= JOINT_4_MAX);

  if (!validA && !validB) {
    // Logger::error("No candidate solution found for point ({}, {}, {}, {}).", t.x, t.y, t.z, t.yaw);
    return false;
  }

  if (validA) {
    c1.q1 = q1;
    c1.q2 = q2_a;
    c1.q3 = q3_a;
    c1.q4 = q4_a;
  }

  if (validB) {
    c2.q1 = q1;
    c2.q2 = q2_b;
    c2.q3 = q3_b;
    c2.q4 = q4_b;
  }

  return true;
}

Vector<TaskPoint> interpolateLine3D(const TaskPoint& start, const TaskPoint& end, const int points) {

  Vector<TaskPoint> interpolatedPoints(points);

  // If the number of steps is less than or equal to 1, return only the start point
  if (points <= 1) {
    interpolatedPoints.pushBack(start);
    return interpolatedPoints;
  }

  for (int i = 0; i <= points; ++i) {
        
    // Interpolation factor t between 0 and 1
    float t = static_cast<float>(i) / points;

    // Compute the interpolated point
    TaskPoint point(
      start.x + t * (end.x - start.x),
      start.y + t * (end.y - start.y),
      start.z + t * (end.z - start.z),
      start.yaw + t * (end.yaw - start.yaw)
    );

    interpolatedPoints.pushBack(point);
  }

  return interpolatedPoints;
}

#endif  // KINEMATICS_HPP
