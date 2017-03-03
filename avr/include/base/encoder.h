#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void initEncoders();

class Encoder {
public:
    Encoder(uint8_t interrupt);
    
    int32_t get();
    int32_t getAndClear();
    
private:
    uint8_t _int;
};

extern Encoder encL;
extern Encoder encR;

#endif //ENCODER_H_
