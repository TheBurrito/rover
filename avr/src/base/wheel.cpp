#include <base/wheel.h>
#include <avr/io.h>
#include <stdlib.h>

Wheel wheelL(&PORTD, &PORTD, 6, 5, &OCR1A, &DDRD, &DDRD);
Wheel wheelR(&PORTB, &PORTD, 0, 7, &OCR1B, &DDRB, &DDRD);

void initMotors() {
    //setup timer1 for motor pwm output
    TCCR1A = 0b10100000; //non-inverted pwm mode, phase & freq correct, top in ICR1
    TCCR1B = 0b00010001; //no prescaler
    ICR1 = MOTOR_MAX; //set PWM resolution
    OCR1A = 0;
    OCR1B = 0;
}

Wheel::Wheel(volatile uint8_t *port1, volatile uint8_t *port2, uint8_t pin1,
        uint8_t pin2, volatile uint16_t *pwm, volatile uint8_t *ddr1,
        volatile uint8_t *ddr2) {
    _port1 = port1;
    _port2 = port2;
    _pin1 = pin1;
    _pin2 = pin2;
    _pwm = pwm;
    
    *ddr1 |= (1<<_pin1);
    *ddr2 |= (1<<_pin2);
    
    if (pwm == &OCR1A) {
        DDRB |= (1<<1);
    } else if (pwm == &OCR1B) {
        DDRB |= (1<<2);
    }
}
    
void Wheel::drive(int val) {
    if (val < 0) {
        val = -val;
        *_port1 |= (1<<_pin1);
        *_port2 &= ~(1<<_pin2);
    } else {
        *_port1 &= ~(1<<_pin1);
        *_port2 |= (1<<_pin2);
    }
    
    *_pwm = val;
}

void Wheel::coast() {
    *_pwm = 0;
}

void Wheel::brake() {
    *_port1 |= (1<<_pin1);
    *_port2 |= (1<<_pin2);
    *_pwm = MOTOR_MAX;
}

/*inline void setPWMA(int val) {
    OCR1A = val;
}

inline void setPWMB(int val) {
    OCR1B = val;
}

inline void setDirA(uint8_t a, uint8_t b) {
    if (a) {
        PORTD |= (1<<6);
    } else {
        PORTD &= ~(1<<6);
    }
    
    if (b) {
        PORTD |= (1<<7);
    } else {
        PORTD &= ~(1<<7);
    }
}

inline void setDirB(uint8_t a, uint8_t b) {
    if (a) {
        PORTB |= (1<<3);
    } else {
        PORTB &= ~(1<<3);
    }
    
    if (b) {
        PORTB |= (1<<4);
    } else {
        PORTB &= ~(1<<4);
    }
}

void driveA(int val) {
    uint8_t dir = val < 0;
    val = abs(val);
    if (val > MOTOR_MAX) val = MOTOR_MAX;
    
    setPWMA(val);
    
    if (dir) {
        setDirA(1, 0);
    } else {
        setDirA(0, 1);
    }
}

void driveB(int val) {
    uint8_t dir = val < 0;
    val = abs(val);
    if (val > MOTOR_MAX) val = MOTOR_MAX;
    
    setPWMB(val);
    
    if (dir) {
        setDirB(1, 0);
    } else {
        setDirB(0, 1);
    }
}

void coast(uint8_t a, uint8_t b) {
    if (a) OCR1A = 0;
    if (b) OCR1B = 0;
}

void brake(uint8_t a, uint8_t b) {
    if (a) {
        setDirA(1, 1);
        setPWMA(MOTOR_MAX);
    }
    
    if (b) {
        setDirB(1, 1);
        setPWMB(MOTOR_MAX);
    }
}*/
