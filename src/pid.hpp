// pid.hpp
//
// A plain PID controller. Used by the control node to turn a heading or
// position error (from the A* planner and the EKF state estimate) into a
// simulated actuator command. There is no real motor or servo here, this
// is a numeric controller only.

#ifndef AUTONOMY_PLATFORM_PID_HPP
#define AUTONOMY_PLATFORM_PID_HPP

using namespace std;

class PidController {
public:
    PidController(float kp, float ki, float kd, float output_limit) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        output_limit_ = output_limit;
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        has_previous_error_ = false;
    }

    // Computes one control output for the given error, over a time step of
    // dt seconds. Includes basic integral clamping to avoid windup.
    float Update(float error, float dt) {
        integral_ = integral_ + error * dt;

        float integral_limit = output_limit_ * 2.0f;
        if (integral_ > integral_limit) {
            integral_ = integral_limit;
        }
        if (integral_ < -integral_limit) {
            integral_ = -integral_limit;
        }

        float derivative = 0.0f;
        if (has_previous_error_ && dt > 0.0f) {
            derivative = (error - previous_error_) / dt;
        }
        previous_error_ = error;
        has_previous_error_ = true;

        float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        if (output > output_limit_) {
            output = output_limit_;
        }
        if (output < -output_limit_) {
            output = -output_limit_;
        }

        return output;
    }

    void Reset() {
        integral_ = 0.0f;
        previous_error_ = 0.0f;
        has_previous_error_ = false;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float output_limit_;
    float integral_;
    float previous_error_;
    bool has_previous_error_;
};

#endif // AUTONOMY_PLATFORM_PID_HPP
