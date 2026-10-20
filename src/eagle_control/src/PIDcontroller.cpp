#include "eagle_control/PIDcontroller.hpp"
#include <algorithm>
#include <iostream>

//NOTE - PIDcontroller Constructor
PIDcontroller::PIDcontroller(const float Kp, const float Ki, const float Kd, const float Min, const float Max) {
    kp = Kp; ki = Ki; kd = Kd; //YAML file for configs
    min = Min; max = Max;
    firstOutput = true;
    iE = 0;
    dE = 0;
}

//NOTE - PIDcontroller::getPIDoutput Function
float PIDcontroller::getPIDoutput(const float target, const float current, const float dt) {
    const float error = target - current;

    updateError(error, dt);

    //Standard PID formula
    float output = kp * error + ki * iE + kd * dE;

    output = std::clamp(output, min, max);

    return output;
}

//NOTE - PIDcontroller::updateError Function
void PIDcontroller::updateError(const float currentError, const float dt) {
    if (!firstOutput && dt > 0) { //dt is already mesured in seconds
        iE += currentError * dt;
        dE = (currentError - pE) / dt;
    } else {
        dE = 0;
        firstOutput = false;
    }

    pE = currentError;
}

//NOTE - PIDcontroller::reset() Function
void PIDcontroller::reset() {
    iE = 0;
    dE = 0;
    firstOutput = true;
}