#pragma once
#include <math.h>

class PID{
public:
    PID(double kp, double ki, double kd);
    double calculate(double setpoint, double measured_value, double dt, double limit);
    double calculate(double error, double dt, double limit);
    void reset(void);
    void setParameters(double kp, double ki, double kd);
private:
    double kp_;
    double ki_;
    double kd_;
    double previous_error_;
    double integral_;
};

