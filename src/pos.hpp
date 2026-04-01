// pos.hpp

#pragma once
#include <cmath>
#include <vector>

/**
 * Constants for the specific robot physical build.
 */
class BotValues {
  public:
    float rightRatio; // Distance per encoder degree
    float leftRatio;
    float trackWidth; // Distance between wheels
    BotValues(float rightRatio, float leftRatio, float trackWidth)
        : rightRatio(rightRatio), leftRatio(leftRatio), trackWidth(trackWidth) {
    }
};

/**
 * Represents the velocity (change in state).
 * API-agnostic: Units are distance/sec and radians/sec.
 */
class Vel {
  public:
    float linear, angular;

    Vel(float linear, float angular) : linear(linear), angular(angular) {}

    // Factory method to calculate velocity from raw hardware readings
    static Vel from_encoders(const BotValues &values, float leftDegPerSec,
                             float rightDegPerSec) {
        float left = leftDegPerSec * values.leftRatio;
        float right = rightDegPerSec * values.rightRatio;
        float lin = (left + right) * 0.5f;
        float ang = (right - left) / values.trackWidth;
        return Vel(lin, ang);
    }
};

class Pos {
  public:
    float x, y, heading; // Heading in radians

    Pos(float x, float y, float heading) : x(x), y(y), heading(heading) {}

    /**
     * Updates the position using an arc-based approximation.
     * newHeading: the raw heading from the IMU in radians
     */
    void apply_with_imu(Vel v, float newHeading, float dt) {
        float dLin = v.linear * dt;

        // We use the IMU's actual change rather than calculating it from motors
        float avgHeading = (heading + newHeading) * 0.5f;

        x += dLin * std::cos(avgHeading);
        y += dLin * std::sin(avgHeading);
        heading = newHeading;

        // Standard normalization
        heading = std::atan2(std::sin(heading), std::cos(heading));
    }
};

/**
 * Pathing Logic: Generates targets for a Tank Drive controller.
 */
class PathLogic {
  public:
    struct Target {
        float linearVel;
        float angularVel;
    };

    /**
     * Calculates required velocities to reach a point.
     * This is a simplified "Point-to-Point" generator.
     */
    static Target step_towards(const Pos &current, float targetX, float targetY,
                               float maxLin, float maxAng) {
        float dx = targetX - current.x;
        float dy = targetY - current.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        // The angle the robot needs to be at to face the point
        float targetAngle = std::atan2(dy, dx);

        // The difference between where we are and where we want to look
        float angleError = targetAngle - current.heading;
        angleError = std::atan2(std::sin(angleError), std::cos(angleError));

        // Simple P-loop logic (You would eventually add PID here)
        float linVel =
            distance > 1.0f ? maxLin : 0; // Move if further than 1 unit
        float angVel = angleError * 2.0f; // Scale turn speed by error

        // Clamp to max values
        if (std::abs(angVel) > maxAng)
            angVel = (angVel > 0 ? 1 : -1) * maxAng;

        return {linVel, angVel};
    }
};

class PID {
  public:
    float kp, ki, kd;
    float error, lastError, integral, derivative;

    PID(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), error(0), lastError(0), integral(0),
          derivative(0) {}

    float calculate(float target, float current, float dt) {
        if (dt <= 0)
            return 0;
        error = target - current;
        integral += error * dt;
        derivative = (error - lastError) / dt;
        lastError = error;

        // Simple integral anti-windup: cap the integral contribution
        if (std::abs(error) < 0.1)
            integral = 0;

        return (error * kp) + (integral * ki) + (derivative * kd);
    }

    void reset() { error = lastError = integral = derivative = 0; }
};
