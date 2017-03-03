#include <base/base.h>
#include <math.h>

#include <system/serial.h>

CBase Base;

template <typename T> bool isNeg(const T &val) {
    return (val < T(0));
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

void normalizeRadians(float *heading) {
    while (*heading < -PI) *heading += TWO_PI;
    while (*heading > PI) *heading -= TWO_PI;
}

CBase::CBase() {
    _driving = false;
    _turning = false;
    _turnDir = true;
    _coast = true;
    _curVel = 0;
    _curTurn = 0;
    _lastEncL = 0;
    _lastEncR = 0;
    
}

void CBase::updateOdometry(const long &encL, const long encR) {
    long diffL = encL - _lastEncL;
    long diffR = encR - _lastEncR;
    
    _lastEncL = encL;
    _lastEncR = encR;
    
    float distL = diffL / _ticksPerUnit;
    float distR = diffR / _ticksPerUnit;
    
    _velL = distL / _dt;
    _velR = distR / _dt;
    
    float forward = (distL + distR) / 2.0;
    _a += (distR - distL) / _width;
    if (_a > PI) {
        _a -= TWO_PI;
    } else if (_a < -PI) {
        _a += TWO_PI;
    }
    
    _x += cos(_a) * forward;
    _y += sin(_a) * forward;
}

void CBase::updateNavigation() {
    //determine the position delta. Used for determining distance and heading
    //to goal as needed
    _dx = _navX - _x;
    _dy = _navY - _y;

    //set dA based on whether the goal heading is specified by radians
    //or by a goal point
    if (_turnDir) {
        _da = _navA - _a;
    } else {
        _da = atan2(_dy, _dx) - _a;
        //printf("a%d\n", (int)(_da * 1000));
    }

    normalizeRadians(&_da);

    //check if turning first
    if (_turning) {
        //check if heading is close enough to goal heading
        if (fabs(_da) < _turnThresh) {
            //we are, turn off turning to allow forward driving if needed
            _turning = false;
        } else {
            //still need to turn, set target turn rate
            _targVel = 0;
            _targTurn = _da;
        }
    }

    //set targets for moving to goal position
    if (!_turning && _driving) {
        //calculate distance to goal to see if we're close enough
        float dist = hypot(_dx, _dy);
        if (dist < _posThresh) {
            //we've reached the goal, turn off driving navigation and stop
            _driving = false;
            _targVel = 0;
            _targTurn = 0;
        } else {
            //set velocity based on goal distance and how aligned we are
            _targVel = cos(_da) * dist;
            
            //always be turning towards goal point
            _targTurn = _da * 2;
        }
    }
}

void CBase::processAcceleration() {
    //setup some intermediate values so we only have to check deadzone
    //on the positive side of zero
    bool velNeg = isNeg(_targVel);
    float absVel = fabs(_targVel);
    
    bool turnNeg = isNeg(_targTurn);
    float absTurn = fabs(_targTurn);
    
    if (absVel < _velDead) absVel = 0;
    else if (absVel > _velMax) absVel = _velMax;
    else if (absVel < _velMin) absVel = _velMin;
    
    if (absTurn < _turnDead) absTurn = 0;
    else if (absTurn > _turnMax) absTurn = _turnMax;
    else if (absTurn < _turnMin) absTurn = _turnMin;
    
    if (velNeg) _targVel = -absVel;
    else        _targVel =  absVel;
    
    if (turnNeg) _targTurn = -absTurn;
    else         _targTurn =  absTurn;
    
    //scaled accel values based on cycle time
    float vAcc = _velAccel * _dt;
    float tAcc = _turnAccel * _dt;
    
    //set current targets based on accel
    if (_targVel < _curVel) {
        //decelerating
        if (_curVel - _targVel > vAcc) _curVel -= vAcc;
        else _curVel = _targVel;
    } else {
        if (_targVel - _curVel > vAcc) _curVel += vAcc;
        else _curVel = _targVel;
    }
    
    if (_targTurn < _curTurn) {
        //turning right
        if (_curTurn - _targTurn > tAcc) _curTurn -= tAcc;
        else _curTurn = _targTurn;
    } else {
        if (_targTurn - _curTurn > tAcc) _curTurn += tAcc;
        else _curTurn = _targTurn;
    }
}

void CBase::updateOutputs() {

    //determine goal velocities for each wheel, factoring in turning
    float turn = 0.5 * _width * _curTurn;
    _targVelL = _curVel - turn;
    _targVelR = _curVel + turn;
    
    _outLeft = (int)_pidL.run(_velL, _targVelL);
    _outRight = (int)_pidR.run(_velR, _targVelR);
    
    bool neg = isNeg(_outLeft);
    _outLeft = fabs(_outLeft);
    
    if (_outLeft < _outDead) _outLeft = 0;
    else if (_outLeft > _outMax) _outLeft = _outMax;
    else if (_outLeft < _outMin) _outLeft = _outMin;
    
    if (neg) _outLeft = -_outLeft;
    
    neg = isNeg(_outRight);
    _outRight = fabs(_outRight);
    
    if (_outRight < _outDead) _outRight = 0;
    else if (_outRight > _outMax) _outRight = _outMax;
    else if (_outRight < _outMin) _outRight = _outMin;
    
    if (neg) _outRight = -_outRight;
}

bool CBase::update(const long &encL, const long &encR) {
    updateOdometry(encL, encR);
    
    //if coasting (no motor control) there is no need to continue
    if (_coast) {
        return true;
    }
    
    //_driving or _turning signify navigation 
    if (_driving || _turning) {
        updateNavigation();
    }
    
    processAcceleration();
    
    updateOutputs();
    
    return (!(_driving || _turning));
}

void CBase::getWheelSetpt(float *left, float *right) {
    *left = _targVelL;
    *right = _targVelR;
}

void CBase::getNavDeltas(float *x, float *y, float *a) {
    *x = _dx;
    *y = _dy;
    *a = _da;
}

void CBase::setAccel(const float &velAccel, const float &turnAccel) {
    _velAccel = velAccel;
    _turnAccel = turnAccel;
}

void CBase::setPID(const float &kp, const float &ki, const float &kd) {
    _pidL.setGains(kp, ki, kd);
    _pidR.setGains(kp, ki, kd);
}

void CBase::setOutRange(const int &deadzone, const int &min, const int &max) {
    _outDead = deadzone;
    _outMin = min;
    _outMax = max;
    
    _pidL.setRange(-max, max);
    _pidR.setRange(-max, max);
}

void CBase::setVelRange(const float &deadzone, const float &min, const float &max) {
    _velDead = deadzone;
    _velMin = min;
    _velMax = max;
}

void CBase::setTurnRange(const float &deadzone, const float &min, const float &max) {
    _turnDead = deadzone;
    _turnMin = min;
    _turnMax = max;
}

void CBase::setThresholds(const float &posThresh, const float &turnThresh) {
    _posThresh = posThresh;
    _turnThresh = turnThresh;
}

void CBase::setWidth(const float &width) {
    _width = width;
}

void CBase::setTicksPerUnit(const float &tpu) {
    _ticksPerUnit = tpu;
}

void CBase::setDeltaTime(const float &dt) {
    _dt = dt;
}

void CBase::setPose(const float &x, const float &y, const float &a) {
    _x = x;
    _y = y;
    _a = a;
}

void CBase::getOutput(int *left, int *right) {
    *left = _outLeft;
    *right = _outRight;
}

void CBase::getTargets(float *vel, float *turn) {
    *vel = _curVel;
    *turn = _curTurn;
}

void CBase::getPose(float *x, float *y, float *a) {
    *x = _x;
    *y = _y;
    *a = _a;
}

void CBase::getVelocity(float *velocity, float *turn) {
    *velocity = _vel;
    *turn = _turn;
}

void CBase::getWheelVel(float *left, float *right) {
    *left = _velL;
    *right = _velR;
}

void CBase::moveTo(const float &x, const float &y, bool turnFirst) {
    _navX = x;
    _navY = y;
    _driving = true;
    _turning = turnFirst;
    _coast = false;
    _turnDir = false;
        
    
    if (turnFirst) {
        printf("Turning to %d, %d\n", (int)(x * 1000), (int)(y * 1000));
    } else {
        printf("Moving to %d, %d\n", (int)(x * 1000), (int)(y * 1000));
    }
}

void CBase::turnTo(const float &x, const float &y) {
    _navX = x;
    _navY = y;
    _driving = false;
    _turning = true;
    _coast = false;
    _turnDir = false;
}

void CBase::turnTo(const float &a) {
    _navA = a;
    _driving = false;
    _turning = true;
    _coast = false;
    _turnDir = true;
}

void CBase::drive(const float &vel, const float &turn) {
    _driving = false;
    _turning = false;
    _coast = false;
    _targVel = vel;
    _targTurn = turn;
}

void CBase::stop() {
    drive(0, 0);
    _coast = false;
}

void CBase::coast() {
    drive(0, 0);
    reset();
    _coast = true;
    _outLeft = 0;
    _outRight = 0;
}

void CBase::reset() {
    _pidL.reset();
    _pidR.reset();
}
