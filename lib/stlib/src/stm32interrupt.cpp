#include <stm32lib.h>
#include <cstdlib>

using namespace st32;

uint32_t timer::milliseconds    = 0;
uint32_t timer::submilliseconds = 0;

uint8_t ProgrammPWM::size = 0;
ProgrammPWM::PWMtasks* ProgrammPWM::tasks = (ProgrammPWM::PWMtasks*)calloc(64, sizeof(Pin));

extern "C" void SysTick_Handler (void) {
    timer::submilliseconds++;

    if (timer::submilliseconds < IntFrequency / 1000) {
        timer::milliseconds++;
        timer::submilliseconds -= IntFrequency / 1000;
    }

    static uint8_t cnt = 0;
    ProgrammPWM::execute(cnt++);
}

void timer::delay_us(uint16_t us) {
    TIM6->CNT = 0;
    while (TIM6->CNT < us);
}
void timer::delay_ms(uint16_t ms) {
    for (uint16_t i = 0; i < ms; i++) delay_us(1000);
}

uint32_t timer::millis () {
    return milliseconds;
}

void Pin::PWMWrite (uint8_t val) const {
    if (val == 0) {
        ProgrammPWM::removePin(*this);
    } else {
        ProgrammPWM::addPin(*this, val);
    }
}
