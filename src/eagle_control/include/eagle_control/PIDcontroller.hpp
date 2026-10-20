#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <chrono>

class PIDcontroller {
    public: 
        PIDcontroller(const float Kp, const float Ki, const float Kd, const float Min, const float Max);

        float getPIDoutput(const float target, const float current, const float dt);

        void updateError(const float currentError, const float dt);

        void reset();

    private:
        float kp, ki, kd, min, max;

        float dE, iE, pE; //Derivate - Integral - PreviousError

        bool firstOutput;
};

#endif