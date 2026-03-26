#include "motion_control/pid.h"

PID::PID(double Kp, double Ki, double Kd) : 
kp_(Kp), ki_(Ki), kd_(Kd), previous_error_(0.0),
integral_(0.0) {}

double PID::calculate(double setpoint, double measured_value, double dt, double limit) {
    double error = setpoint - measured_value;
    integral_ += error * dt;
    double derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    return fmin(fmax(output, -limit), limit);
}

double PID::calculate(double error, double dt, double limit) {
    integral_ += error * dt;
    double derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    return fmin(fmax(output, -limit), limit);
}

void PID::reset() {
    previous_error_ = 0.0;
    integral_ = 0.0;
}

void PID::setParameters(double Kp, double Ki, double Kd) {
    kp_ = Kp;
    ki_ = Ki;
    kd_ = Kd;
}
