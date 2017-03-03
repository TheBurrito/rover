#include <base/base.h>
#include <base/encoder.h>
#include <base/wheel.h>

#include <system/timer.h>
#include <system/serial.h>

#include <avr/io.h>
#include <avr/interrupt.h>

typedef struct {
    float x;
    float y;
} Point2f;

Point2f path[] = {{0.8, 0.0}, {0.0, 0.0}};

void printDec(float f) {
    printf("%d.%d", (int)f, (int)(f * 1000) % 1000);
}

void init() {
    initSerial(57600);
    initTimer();
    initEncoders();
    initMotors();
    
    Base.setAccel(0.5, PI);
    Base.setPID(16000, 1000, 500);
    Base.setOutRange(4000, 0, 0x7fff);
    Base.setVelRange(0.01, 0.05, 0.2);
    Base.setTurnRange(0.01, 0.2, 3.0);
    
    Base.setThresholds(0.01, 0.01);
    Base.setWidth(0.17055);
    Base.setTicksPerUnit((100 * 2 * 30) / (0.1234 * PI));
    Base.setDeltaTime(0.01);
    
    DDRB |= (1<<5);
}

int main() {
    init();
    
    sei();
    
    unsigned long curMillis = millis();
    unsigned long lastDrive = curMillis;
    unsigned long drivePeriod = 10;
    
    unsigned long lastOdom = curMillis;
    unsigned long odomPeriod = 500;
    
    int outLeft, outRight;
    float x, y, a;
    float tVel, tTurn;
    float dx, dy, da;
    float vl, vr;
    
    bool led = false;
    
    int32_t countL, countR;
    
    Base.drive(0.0, 0.0);
    //Base.coast();
    
    int curPt = -1;
    
    while (1) {
        curMillis = millis();
        
        if (curMillis - lastDrive >= drivePeriod) {
            lastDrive += drivePeriod;
            
            countL = encL.get();
            countR = encR.get();
            
            if (Base.update(countL, countR)) {
                curPt = (curPt + 1) % 2;
                Base.moveTo(path[curPt].x, path[curPt].y, true);
            }
            
            Base.getOutput(&outLeft, &outRight);
            wheelL.drive(outLeft);
            wheelR.drive(outRight);
            
            //turn on the LED if any motor output hits 100%
            if (outLeft == 0x7fff || outLeft == -0x7fff || outRight == 0x7fff || outRight == -0x7fff) {
                led = true;
            } else {
                led = false;
            }
        
            if (led) PORTB |= (1<<5);
            else PORTB &= ~(1<<5);
        }
        
        if (curMillis - lastOdom >= odomPeriod) {
            lastOdom += odomPeriod;
            
            Base.getPose(&x, &y, &a);
            Base.getTargets(&tVel, &tTurn);
            Base.getNavDeltas(&dx, &dy, &da);
            Base.getWheelSetpt(&vl, &vr);
            printf("\tPose: %d, %d, %d\n", (int)(x * 1000), (int)(y * 1000), (int)(a * 572.9578));
            printf("\tDelt: %d, %d, %d\n", (int)(dx * 1000), (int)(dy * 1000), (int)(da * 572.9578));
            printf("\tTarg: %d, %d\n", (int)(tVel * 1000), (int)(tTurn * 100));
            printf("\tWhl: %d, %d\n", (int)(vl * 1000), (int)(vr * 1000));
        }
    }
}
