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

struct PathPoint {
    float x, y, heading;
    bool reverse; // If true, the robot drives butt-first to this point
};
class PathFollower {
  private:
    float lookaheadDist = 0.4f; // How far ahead to look (meters)
    float trackWidth;
    PID &velPID; // Now we use PID to control actual velocity, not just voltage

  public:
    PathFollower(float tw, PID &v) : trackWidth(tw), velPID(v) {}

    struct DriveVel {
        float left, right;
    };

    DriveVel update(const Pos &current, const PathPoint &target, float maxVel) {
        // 1. Transform target to Robot-Local Coordinates
        float dx = target.x - current.x;
        float dy = target.y - current.y;

        // Rotate the global offset into the robot's local frame
        // Local X is "side to side", Local Y is "forward/back"
        float localX =
            dx * std::sin(current.heading) - dy * std::cos(current.heading);
        float localY =
            dx * std::cos(current.heading) + dy * std::sin(current.heading);

        float L2 = dx * dx + dy * dy; // Lookahead distance squared
        float dist = std::sqrt(L2);

        // 2. Calculate Curvature (K)
        // K = 2x / L^2. This is the inverse of the radius of the arc.
        float curvature = (2.0f * localX) / L2;

        // 3. Differential Drive Kinematics
        // Left = V * (2 + K*W) / 2
        // Right = V * (2 - K*W) / 2
        float v = maxVel;

        // Slow down if we are far from the target heading (High curvature)
        if (std::abs(curvature) > 1.0f)
            v /= std::abs(curvature);

        float leftVel = v * (2.0f + curvature * trackWidth) / 2.0f;
        float rightVel = v * (2.0f - curvature * trackWidth) / 2.0f;

        // If reversing, flip the logic
        if (target.reverse) {
            std::swap(leftVel, rightVel);
            leftVel *= -1;
            rightVel *= -1;
        }

        return {leftVel, rightVel};
    }
};
