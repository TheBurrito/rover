#ifndef PID_H_
#define PID_H_

class PID {
public:
    PID();
    PID(const float &kp, const float &ki, const float &kd);
    
    void setGains(const float &kp, const float &ki, const float &kd);
    void setRange(const float &min, const float &max);
    
    float run(const float &value, const float &setpoint);
    
    void reset();
    
private:
    float _kp;
    float _ki;
    float _kd;
    
    float _min;
    float _max;
    
    float _sum;
    float _last;
    
    bool _reset;
};

#endif //PID_H_
