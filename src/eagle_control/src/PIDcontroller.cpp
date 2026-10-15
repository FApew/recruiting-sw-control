#include "eagle_control/PIDcontroller.hpp"
#include <algorithm>

PIDcontroller::PIDcontroller(const float Kp, const float Ki, const float Kd, const float Min, const float Max) {
    kp = Kp; ki = Ki; kd = Kd;
    min = Min; max = Max;
    firstOutput = true;
    iE = 0;
    dE = 0;
}

float PIDcontroller::getPIDoutput(const float target, const float current, const float dt) {
    const float error = target - current;

    updateError(error, dt);

    float output = kp * error + ki * iE + kd * dE;

    output = std::clamp(output, min, max);

    return output;
}

void PIDcontroller::updateError(const float currentError, const float dt) {
    if (!firstOutput) {
        iE += currentError * dt;
        dE = (currentError - pE) / dt;
    } else {
        dE = 0;
        firstOutput = false;
    }

    pE = currentError;
}

void PIDcontroller::reset() {
    iE = 0;
    dE = 0;
    firstOutput = true;
}