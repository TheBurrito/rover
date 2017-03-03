#ifndef BASE_H_
#define BASE_H_

//includes functions for velocity control and odometry

//these defines establish the geometry used for calculating odometry

//width of the wheelcenters in meters
#define WHEELBASE 0.17055

//yummmm
#define PI 3.1415926536
#define TWO_PI (PI * 2.0)

//ideal encoder counts per meter
//100cpr encoder in 2x mode with a 30:1 gearbox and a wheel diameter of 0.1234m
#define TICKSPERMETER ((100 * 2 * 30) / (0.1234 * PI))

//Period in ms for calculating odometry and velocity controls
#define ODOM_PERIOD 10

#include <util/pid.h>

class CBase {
public:
    CBase();
    
    void setAccel(const float &velAccel, const float &turnAccel);
    
    void setPID(const float &kp, const float &ki, const float &kd);
    
    void setOutRange(const int &deadzone, const int &min, const int &max);
    void setVelRange(const float &deadzone, const float &min, const float &max);
    void setTurnRange(const float &deadzone, const float &min, const float &max);
    
    void setThresholds(const float &posThresh, const float &turnThresh);
    
    void setWidth(const float &width);
    void setTicksPerUnit(const float &tpu);
    
    void setDeltaTime(const float &dt);
    
    void setPose(const float &x, const float &y, const float &a);
    
    bool update(const long &encL, const long &encR);
    
    void getOutput(int *left, int *right);
    void getPose(float *x, float *y, float *a);
    void getVelocity(float *velocity, float *turn);
    void getWheelVel(float *left, float *right);
    
    //control methods
    void moveTo(const float &x, const float &y, bool turnFirst);

    void turnTo(const float &x, const float &y);
    void turnTo(const float &a);
    
    void drive(const float &vel, const float &turn);
    
    void getTargets(float *vel, float *turn);
    void getWheelSetpt(float *left, float *right);
    void getNavDeltas(float *x, float *y, float *a);
    
    void stop();
    void coast();
    void reset();
    
private:
    //output variables
    int _outMax, _outDead, _outMin;
    int _outLeft, _outRight;
    float _targVelL, _targVelR;
    
    //velocity control variables
    float _velMax, _velDead, _velMin, _velAccel;
    float _turnMax, _turnDead, _turnMin, _turnAccel;
    
    bool _coast;
    
    //odometry variables
    float _ticksPerUnit, _width;
    long _lastEncL, _lastEncR;
    float _x, _y, _a;
    
    //measured values
    float _velL, _velR;
    float _vel, _turn;
    
    //navigation variables
    float _navX, _navY, _navA;
    bool _turnFirst, _driving, _turning, _turnDir;
    float _posThresh, _turnThresh;
    float _dx, _dy, _da;
    
    float _targVel, _targTurn;
    float _curVel, _curTurn;
    
    float _dt;
        
    PID _pidL, _pidR;
    
    void updateOdometry(const long &encL, const long encR);
    void updateNavigation();
    void processAcceleration();
    void updateOutputs();
};

extern CBase Base;

#endif //BASE_H_
