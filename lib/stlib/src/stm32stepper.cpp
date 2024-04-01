#include <stm32stepper.h>

using namespace st32;

Stepper::Stepper (const Pin& in1, const Pin& in2, const Pin& in3, const Pin& in4, Direction dir)
    : in1(in1), in2(in2), in3(in3), in4(in4), currentPhase(0), currentDir(dir) {    
    in1.setDigitalOutput();
    in2.setDigitalOutput();
    in3.setDigitalOutput();
    in4.setDigitalOutput();
}

void Stepper::writePhase (uint8_t phmsk) {
    in1.digitalWrite((phmsk >> (uint8_t)0) & 1);
    in2.digitalWrite((phmsk >> (uint8_t)1) & 1);
    in3.digitalWrite((phmsk >> (uint8_t)2) & 1);
    in4.digitalWrite((phmsk >> (uint8_t)3) & 1);
}

void Stepper::step () {
    currentPhase = (currentPhase + static_cast<uint8_t>(currentDir)) % STPS_CNT;
    writePhase(phases[currentPhase]);
}

void Stepper::setDirection (Direction dir) {
    currentDir = dir;
}