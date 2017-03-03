#include <base/encoder.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include <stdio.h>

Encoder encL(0);
Encoder encR(1);

volatile int32_t _enc[2];

uint8_t port;
bool a, b;

ISR(INT0_vect) {
    a = (PIND & (1<<2)) > 0;
    b = (PINC & (1<<1)) > 0;
    
    if (a == b) {
        _enc[0]++;
    } else {
        _enc[0]--;
    }
}

ISR(INT1_vect) {
    a = (PIND & (1<<3)) > 0;
    b = (PINC & (1<<2)) > 0;
    
    if (a == b) {
        _enc[1]--;
    } else {
        _enc[1]++;
    }
}

void initEncoders() {
    EICRA |= 5; //set both external interrupts to level change
    EIMSK |= 3; //enable both external interrupts
}

Encoder::Encoder(uint8_t interrupt) {
    _int = interrupt;
    
    uint8_t oldSreg = SREG;
    cli();
    
    _enc[_int] = 0;
    
    SREG = oldSreg;
}

int32_t Encoder::get() {
    uint8_t oldSreg = SREG;
    cli();
    int32_t count = _enc[_int];
    SREG = oldSreg;
    
    return count;
}

int32_t Encoder::getAndClear() {
    uint8_t oldSreg = SREG;
    cli();
    int32_t count = _enc[_int];
    _enc[_int] = 0;
    SREG = oldSreg;
    
    return count;
}
