#pragma once
#ifndef STM32STEPPER_h
#define STM32STEPPER_h

#include <stm32lib.h>

namespace st32 {

class Motor {};
class Stepper : public Motor {
public:
    enum class Direction : uint8_t {
        CW  = 0x01,
        CCW = 0xff,
    };
private:
    Pin in1;
    Pin in2;
    Pin in3;
    Pin in4;

    #define STPS_CNT (uint8_t)8
    const uint8_t phases[STPS_CNT] = {0b1001, 0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000};

    uint8_t   currentPhase = 0;
    Direction currentDir;

    void writePhase (uint8_t phmsk);
public:
    Stepper (const Pin& in1, const Pin& in2, const Pin& in3, const Pin& in4, Direction dir = Direction::CW);

    void step ();
    void setDirection (Direction dir);
};

}

#endif // stm32stepper.h