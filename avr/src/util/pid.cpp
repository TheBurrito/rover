#include <util/pid.h>

PID::PID() {
    setGains(0.0, 0.0, 0.0);
    reset();
    _min = 0.0;
    _max = 100.0;
}

PID::PID(const float &kp, const float &ki, const float &kd) {
    setGains(kp, ki, kd);
    reset();
    _min = 0.0;
    _max = 100.0;
}

void PID::setGains(const float &kp, const float &ki, const float &kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::setRange(const float &min, const float &max) {
    _min = min;
    _max = max;
}

float PID::run(const float &value, const float &setpoint) {
    float error = setpoint - value;
    
    _sum += error * _ki;
    
    if (_sum > _max) _sum = _max;
    else if (_sum < _min) _sum = _min;
    
    float output = _kp * error + _sum;
    
    if (_reset) {
        _reset = false;
    } else {
        output -= (value - _last) * _kd;
    }
    
    _last = value;
    
    if (output > _max) output = _max;
    else if (output < _min) output = _min;
    
    return output;
}

void PID::reset() {
    _reset = true;
    _sum = 0;
}
